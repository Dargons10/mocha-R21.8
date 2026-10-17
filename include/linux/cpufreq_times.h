/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_CPUFREQ_TIMES_H
#define _LINUX_CPUFREQ_TIMES_H
#include <asm/cputime.h>
#include <linux/types.h>
struct task_struct;
struct seq_file;
struct cpufreq_policy;
struct cpufreq_frequency_table;
#ifdef CONFIG_CPU_FREQ_TIMES
void cpufreq_times_register(struct cpufreq_policy *, struct cpufreq_frequency_table *);
void cpufreq_times_transition(unsigned int cpu, unsigned int frequency);
void cpufreq_uid_times_remove(uid_t first, uid_t last);
void cpufreq_task_times_free(struct task_struct *);
void cpufreq_task_times_account(struct task_struct *, cputime_t);
int cpufreq_task_times_show(struct seq_file *, struct task_struct *);
#else
static inline void cpufreq_uid_times_remove(uid_t first, uid_t last) {}
static inline void cpufreq_times_register(struct cpufreq_policy *p, struct cpufreq_frequency_table *t) {}
static inline void cpufreq_times_transition(unsigned int cpu, unsigned int freq) {}
static inline void cpufreq_task_times_free(struct task_struct *t) {}
static inline void cpufreq_task_times_account(struct task_struct *t, cputime_t time) {}
#endif
#endif
