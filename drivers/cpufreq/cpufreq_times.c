// SPDX-License-Identifier: GPL-2.0
/* Per-task CPU execution time at each frequency. Units exported to proc are
 * USER_HZ ticks, matching task utime/stime and Android's KernelCpuThreadReader.
 * This is accounting only: no governor, frequency or affinity changes. */
#include <linux/cpufreq.h>
#include <linux/cpufreq_times.h>
#include <linux/list.h>
#include <linux/hashtable.h>
#include <linux/percpu.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/ratelimit.h>
#include <linux/proc_fs.h>
#include <linux/cred.h>
#include <linux/user_namespace.h>

/* Keep task_struct layout unchanged for stock binary kernel modules. */
struct task_frequency_times {
	struct hlist_node node;
	struct task_struct *task;
	unsigned int count;
	u64 *times;
};
struct frequency_entry {
	struct list_head list;
	unsigned int frequency, index;
};
struct cpu_frequency_state {
	cpumask_t related;
	unsigned int index; /* one-based; zero means cpufreq is not initialized */
};
static LIST_HEAD(frequencies);
static unsigned int frequency_count;
static DEFINE_SPINLOCK(times_lock);
static DEFINE_HASHTABLE(task_times, 8);
struct uid_frequency_times {
	struct hlist_node node;
	uid_t uid;
	unsigned int count;
	u64 *times;
};
static DEFINE_HASHTABLE(uid_times, 8);
static unsigned int uid_count;

/* UID counters survive task exit and follow the UID at accounting time. */
static void account_uid(uid_t uid, unsigned int index, cputime_t time)
{
	struct uid_frequency_times *entry;
	unsigned int old_count;
	hash_for_each_possible(uid_times, entry, node, uid)
		if (entry->uid == uid)
			goto found;
	entry = kzalloc(sizeof(*entry), GFP_ATOMIC);
	if (!entry)
		goto fail;
	entry->uid = uid;
	hash_add(uid_times, &entry->node, uid);
	uid_count++;
found:
	old_count = entry->count;
	if (old_count < frequency_count) {
		u64 *values = krealloc(entry->times, frequency_count * sizeof(u64), GFP_ATOMIC);
		if (!values)
			goto fail;
		memset(values + old_count, 0, (frequency_count - old_count) * sizeof(u64));
		entry->times = values;
		entry->count = frequency_count;
	}
	entry->times[index - 1] += time;
	return;
fail:
	pr_warn_ratelimited("cpufreq times: cannot allocate UID counters\n");
}

void cpufreq_uid_times_remove(uid_t first, uid_t last)
{
	struct uid_frequency_times *entry;
	struct hlist_node *tmp;
	unsigned int bucket;
	unsigned long flags;
	spin_lock_irqsave(&times_lock, flags);
	hash_for_each_safe(uid_times, bucket, tmp, entry, node) {
		if (entry->uid < first || entry->uid > last)
			continue;
		hash_del(&entry->node);
		uid_count--;
		kfree(entry->times);
		kfree(entry);
	}
	spin_unlock_irqrestore(&times_lock, flags);
}

/* Copy under the accounting lock, then format without holding it. */
static int uid_times_show(struct seq_file *m, void *unused)
{
	struct uid_frequency_times *entry;
	struct frequency_entry *freq;
	unsigned int rows, cols, actual_rows, actual_cols, bucket, r, c;
	unsigned long flags;
	u64 *values;
	unsigned int *freqs;
	int attempt;
	for (attempt = 0; attempt < 3; attempt++) {
		spin_lock_irqsave(&times_lock, flags);
		rows = uid_count + 8;
		cols = frequency_count + 8;
		spin_unlock_irqrestore(&times_lock, flags);
		values = kcalloc(rows, (cols + 1) * sizeof(u64), GFP_KERNEL);
		freqs = kcalloc(cols, sizeof(*freqs), GFP_KERNEL);
		if (!values || !freqs) {
			kfree(values);
			kfree(freqs);
			return -ENOMEM;
		}
		spin_lock_irqsave(&times_lock, flags);
		if (uid_count > rows || frequency_count > cols) {
			spin_unlock_irqrestore(&times_lock, flags);
			kfree(values);
			kfree(freqs);
			continue;
		}
		actual_cols = 0;
		list_for_each_entry(freq, &frequencies, list)
			freqs[actual_cols++] = freq->frequency;
		actual_rows = 0;
		hash_for_each(uid_times, bucket, entry, node) {
			u64 *row = values + actual_rows++ * (cols + 1);
			row[0] = entry->uid;
			c = 1;
			list_for_each_entry(freq, &frequencies, list) {
				row[c++] = freq->index <= entry->count ?
					cputime64_to_clock_t(entry->times[freq->index - 1]) : 0;
			}
		}
		spin_unlock_irqrestore(&times_lock, flags);
		seq_puts(m, "uid:");
		for (c = 0; c < actual_cols; c++)
			seq_printf(m, " %u", freqs[c]);
		seq_putc(m, '\n');
		for (r = 0; r < actual_rows; r++) {
			u64 *row = values + r * (cols + 1);
			seq_printf(m, "%u:", (unsigned int)row[0]);
			for (c = 1; c <= actual_cols; c++)
				seq_printf(m, " %llu", (unsigned long long)row[c]);
			seq_putc(m, '\n');
		}
		kfree(values);
		kfree(freqs);
		return 0;
	}
	return -EAGAIN;
}
static int uid_times_open(struct inode *inode, struct file *file)
{
	return single_open(file, uid_times_show, NULL);
}
static const struct file_operations uid_times_fops = {
	.open = uid_times_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
static int __init uid_times_init(void)
{
	return proc_create("uid_time_in_state", 0444, NULL, &uid_times_fops) ? 0 : -ENOMEM;
}
late_initcall(uid_times_init);

static DEFINE_PER_CPU(struct cpu_frequency_state, cpu_frequency_state);

/* Called with times_lock held; indexes remain stable across CPU hotplug. */
static unsigned int frequency_index(unsigned int frequency)
{
	struct frequency_entry *e, *pos;
	if (!frequency)
		return 0;
	list_for_each_entry(e, &frequencies, list)
		if (e->frequency == frequency)
			return e->index;
	e = kzalloc(sizeof(*e), GFP_ATOMIC);
	if (!e) {
		pr_warn_ratelimited("cpufreq times: cannot allocate frequency entry\n");
		return 0;
	}
	e->frequency = frequency;
	e->index = ++frequency_count;
	list_for_each_entry(pos, &frequencies, list) {
		if (pos->frequency > frequency) {
			list_add_tail(&e->list, &pos->list);
			return e->index;
		}
	}
	list_add_tail(&e->list, &frequencies);
	return e->index;
}
/* Android caches the frequency columns once. Account transient PLL rates in
 * the nearest supported policy bin, rather than inserting new columns and
 * changing the meaning of existing readers' array indexes. No CPU time is
 * discarded and no clock/governor setting is changed. times_lock is held. */
static unsigned int policy_frequency_index(unsigned int frequency)
{
	struct frequency_entry *entry;
	unsigned int index = 0, distance = UINT_MAX;
	list_for_each_entry(entry, &frequencies, list) {
		unsigned int delta = entry->frequency > frequency ?
			entry->frequency - frequency : frequency - entry->frequency;
		if (delta < distance) {
			distance = delta;
			index = entry->index;
		}
	}
	return index;
}
void cpufreq_times_register(struct cpufreq_policy *policy,
			    struct cpufreq_frequency_table *table)
{
	unsigned long flags;
	unsigned int i, cpu, index;
	spin_lock_irqsave(&times_lock, flags);
	for (i = 0; table[i].frequency != CPUFREQ_TABLE_END; i++)
		if (table[i].frequency != CPUFREQ_ENTRY_INVALID)
			frequency_index(table[i].frequency);
	index = policy_frequency_index(policy->cur);
	for_each_cpu(cpu, policy->related_cpus) {
		cpumask_copy(&per_cpu(cpu_frequency_state, cpu).related, policy->related_cpus);
		per_cpu(cpu_frequency_state, cpu).index = index;
	}
	spin_unlock_irqrestore(&times_lock, flags);
}
void cpufreq_times_transition(unsigned int cpu, unsigned int frequency)
{
	unsigned long flags;
	unsigned int related, index;
	spin_lock_irqsave(&times_lock, flags);
	index = policy_frequency_index(frequency);
	for_each_cpu(related, &per_cpu(cpu_frequency_state, cpu).related)
		per_cpu(cpu_frequency_state, related).index = index;
	spin_unlock_irqrestore(&times_lock, flags);
}
static struct task_frequency_times *find_task_times(struct task_struct *task)
{
	struct task_frequency_times *entry;
	hash_for_each_possible(task_times, entry, node, (unsigned long)task)
		if (entry->task == task)
			return entry;
	return NULL;
}
void cpufreq_task_times_free(struct task_struct *task)
{
	struct task_frequency_times *entry;
	unsigned long flags;
	spin_lock_irqsave(&times_lock, flags);
	entry = find_task_times(task);
	if (entry)
		hash_del(&entry->node);
	spin_unlock_irqrestore(&times_lock, flags);
	if (entry) {
		kfree(entry->times);
		kfree(entry);
	}
}
void cpufreq_task_times_account(struct task_struct *task, cputime_t time)
{
	struct task_frequency_times *times;
	unsigned long flags;
	unsigned int index, old_count;
	if (!task->pid)
		return;
	spin_lock_irqsave(&times_lock, flags);
	index = per_cpu(cpu_frequency_state, task_cpu(task)).index;
	if (!index)
		goto out;
	account_uid(from_kuid_munged(&init_user_ns, task_uid(task)), index, time);
	times = find_task_times(task);
	if (!times) {
		times = kzalloc(sizeof(*times), GFP_ATOMIC);
		if (!times) {
			pr_warn_ratelimited("cpufreq times: cannot allocate task entry\n");
			goto out;
		}
		times->task = task;
		hash_add(task_times, &times->node, (unsigned long)task);
	}
	old_count = times->count;
	if (old_count < frequency_count) {
		u64 *values = krealloc(times->times, frequency_count * sizeof(u64), GFP_ATOMIC);
		if (!values) {
			pr_warn_ratelimited("cpufreq times: cannot allocate task counters\n");
			goto out;
		}
		memset(values + old_count, 0, (frequency_count - old_count) * sizeof(u64));
		times->times = values;
		times->count = frequency_count;
	}
	times->times[index - 1] += time;
out:
	spin_unlock_irqrestore(&times_lock, flags);
}
int cpufreq_task_times_show(struct seq_file *m, struct task_struct *task)
{
	struct frequency_entry *e;
	struct task_frequency_times *times;
	unsigned long flags;
	spin_lock_irqsave(&times_lock, flags);
	times = find_task_times(task);
	list_for_each_entry(e, &frequencies, list) {
		u64 time = times && e->index <= times->count ? times->times[e->index - 1] : 0;
		seq_printf(m, "%u %llu\n", e->frequency,
			   (unsigned long long)cputime64_to_clock_t(time));
	}
	spin_unlock_irqrestore(&times_lock, flags);
	return 0;
}
