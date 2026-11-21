/*
 * drivers/video/tegra/host/nvhost_channel.c
 *
 * Tegra Graphics Host Channel
 *
 * Copyright (c) 2010-2015, NVIDIA Corporation.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "nvhost_channel.h"
#include "dev.h"
#include "nvhost_acm.h"
#include "nvhost_job.h"
#include "nvhost_hwctx.h"
#include "chip_support.h"

#include <trace/events/nvhost.h>
#include <linux/nvhost_ioctl.h>
#include <linux/nvhost.h>
#include <linux/slab.h>
/* mocha K-76b: includes explicitos para el rescate y su boton sysfs. */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/jiffies.h>
#include <linux/atomic.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/string.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>

/* mocha K-76b: el envoltorio publico llama al cuerpo interno con el marcador de reintento. */
static int nvhost_channel_map_interno(struct nvhost_device_data *pdata,
			struct nvhost_channel **channel, int ya_rescatado);

#define NVHOST_CHANNEL_LOW_PRIO_MAX_WAIT 50

/* Constructor for the host1x device list */
int nvhost_channel_list_init(struct nvhost_master *host)
{
	if (host->info.nb_channels > BITS_PER_LONG) {
		WARN(1, "host1x hardware has more channels than supported\n");
		return -ENOSYS;
	}

	host->chlist = kzalloc(host->info.nb_channels *
				sizeof(struct nvhost_channel *), GFP_KERNEL);
	if (host->chlist == NULL)
			return -ENOMEM;

	mutex_init(&host->chlist_mutex);

	return 0;
}

/* Memory allocation for all supported channels */
int nvhost_alloc_channels(struct nvhost_master *host)
{
	int max_channels = host->info.nb_channels;
	int i, err = 0;
	struct nvhost_channel *ch;

	err = nvhost_channel_list_init(host);
	if (err) {
		dev_err(&host->dev->dev, "failed to init channel list\n");
		return err;
	}

	mutex_lock(&host->chlist_mutex);
	for (i = 0; i < max_channels; i++) {
		ch = nvhost_alloc_channel_internal(i, max_channels);
		if (!ch) {
			dev_err(&host->dev->dev, "failed to alloc channels\n");
			mutex_unlock(&host->chlist_mutex);
			return -ENOMEM;
		}
		host->chlist[i] = ch;
		ch->dev = NULL;
		ch->chid = NVHOST_INVALID_CHANNEL;
	}
	mutex_unlock(&host->chlist_mutex);

	return 0;
}

/* return any one of assigned channel from device
 * This API can be used to check if any channel assigned to device
 */
struct nvhost_channel *nvhost_check_channel(struct nvhost_device_data *pdata)
{
	int i;
	struct nvhost_channel *ch;

	for (i = 0; i < pdata->num_channels; i++) {
		ch = pdata->channels[i];
		if (ch && ch->chid != NVHOST_INVALID_CHANNEL)
			return ch;
	}

	return NULL;
}

/* Check if more than channel needed for device and assign */
int nvhost_channel_assign(struct nvhost_device_data *pdata,
			  struct nvhost_channel *ch)
{
	int i;

	for (i = 0; i < pdata->num_channels; i++) {
		if (!pdata->channels[i]) {
			pdata->channels[i] = ch;
			pdata->num_mapped_chs++;
			ch->dev_chid = i;
			return 0;
		}
	}
	dev_err(&pdata->pdev->dev, "%s: All channels assigned\n", __func__);

	return -EINVAL;
}

/* Releases all channels assigned with device */
int nvhost_channel_release(struct nvhost_device_data *pdata)
{
	struct nvhost_channel *ch;
	int i;

	for (i = 0; i < pdata->num_channels; i++) {
		ch = pdata->channels[i];
		if (ch && ch->dev)
			nvhost_putchannel(ch, 1);
	}
	return 0;
}
/* Unmap channel from device and free all resources, deinit device */
static int nvhost_channel_unmap_locked(struct nvhost_channel *ch)
{
	struct nvhost_device_data *pdata;
	struct nvhost_master *host;
	int max_channels;
	int i = 0;

	if (!ch->dev) {
		pr_err("%s: freeing unmapped channel\n", __func__);
		return 0;
	}

	pdata = platform_get_drvdata(ch->dev);
	host = nvhost_get_host(pdata->pdev);

	max_channels = host->info.nb_channels;

	if (ch->chid == NVHOST_INVALID_CHANNEL) {
		dev_err(&host->dev->dev, "Freeing un-mapped channel\n");
		return 0;
	}
	if (ch->error_notifier_ref)
		nvhost_free_error_notifiers(ch);

	dev_dbg(&ch->dev->dev, "channel %d un-mapped\n", ch->chid);

	pdata->num_mapped_chs--;

	/* Allow keep-alive'd module to be turned off
	 * make sure that all channels are unmapped before calling
	 * nvhost_module_enable_poweroff
	 */
	if (!pdata->num_mapped_chs) {
		channel_cdma_op().stop(&ch->cdma);
		nvhost_cdma_deinit(&ch->cdma);

		if (pdata->keepalive)
			nvhost_module_enable_poweroff(pdata->pdev);

		/*
		 * when ALL of the channels are unmapped from device,
		 * we can free all the host managed syncpts assigned
		 * to that device
		 */
		for (i = 0; i < NVHOST_MODULE_MAX_SYNCPTS; ++i) {
			if (pdata->syncpts[i]) {
				nvhost_free_syncpt(pdata->syncpts[i]);
				pdata->syncpts[i] = 0;
			}
		}
	}

	clear_bit(ch->chid, &host->allocated_channels);

	ch->chid = NVHOST_INVALID_CHANNEL;
	ch->dev = NULL;
	kfree(ch->ctxhandler);
	ch->ctxhandler = NULL;
	ch->cur_ctx = NULL;
	ch->aperture = NULL;
	ch->refcount = 0;
	pdata->channels[ch->dev_chid] = NULL;

	return 0;
}

/*
 * mocha K-76b (2026-11-21): rescate del canal exclusivo huerfano.
 *
 * Prueba medida en mocha (T124), arranque del 2026-11-21 registrada en s.log:
 *   19:22:09  'vi.0: nvhost_channelctl: allocated waitbase 7 for vi.0[0]'   (probe/power_on de
 *             imx179 y ov5693: el kernel asigna y mapea los canales VI/ISP al arrancar)
 *   19:25:56  'bq27x00 BMS ... v 3831000 i -294000'   (bateria SANA y cargando: el fallo NO es
 *             electrico en este punto)
 *   19:26:35  'mocha K-76a: vi.1 exclusivo ocupado; canal 0 chid 10 refcount 1 cur_ctx 00000000
 *             cdma: slots_used 2 ... running 1 torndown 0'  +  num_mapped_chs=1 exclusive=1
 *             keepalive=1'
 *   19:26:35  '__nvhost_channelopen: failed to map channel, error: -16'   (-EBUSY)
 *   19:26:44  'cdma_timeout_handler: timeout: 22 (vi.1_2) ctx 0xdb663800, HW thresh 0, done 1'
 *   19:26:57  'Camera 0: initialize: Could not open camera session: Invalid argument (-22)'
 * Es decir: el trabajo que tomo la referencia del canal exclusivo nunca cerro, el desmapeo solo
 * ocurre cuando refcount llega a 0 (nvhost_putchannel -> nvhost_channel_unmap_locked) y por tanto
 * el dispositivo de camara quedaba ciego para siempre. Eso es lo que explica el patron del usuario
 * 'funcionaba en el arranque, tras el reinicio ya no': el reinicio limpia el estado del kernel, el
 * probe vuelve a dejar el canal mapeado y la primera apertura posterior devuelve -EBUSY.
 *
 * REVISTA DEL INTENTO ANTERIOR (medido el 2026-11-21, hay que dejarlo escrito):
 * La primera version del rescate se disparaba cuando el trabajo pendiente llevaba dos plazos de
 * timeout vencidos, SIN mirar si el CDMA seguia cargando. Con el estado real que se midio
 * ('running 1', 'slots_used 2', cola de sincronizacion llena) eso llamaba a nvhost_putchannel ->
 * nvhost_channel_unmap_locked -> channel_cdma_op().stop + nvhost_cdma_deinit, que libera el
 * pushbuffer MIENTRAS el DMA de host1x todavia lo esta leyendo. Resultado: el kernel se colgo a
 * media arranque y el LK se queda en APX / fastboot. Recuperable escribiendo el bloque bueno con
 * 'fastboot flash boot' (en este LK la particion se llama 'boot', NO 'LNX').
 *
 * Por eso el rescate exige, TODOS a la vez:
 *   - cdma.running == false        : el DMA del canal esta parado (nadie leyendo el pushbuffer)
 *   - sync_queue vacia             : no queda ningun trabajo en vuelo
 *   - cur_ctx == NULL              : ningun cliente es dueno del contexto del canal
 *   - refcount > 0                 : queda la referencia colgada que hay que soltar
 *   - el nombre del dispositivo empieza por 'vi' o 'isp' (solo los modulos de camara, que son los
 *     exclusivos con num_channels = 1; no se toca gr2d, vic, tsec, ni nvjpg)
 *   - el sistema ya termino de arrancar (mocha_pronto, puesto por un late_initcall): durante el
 *     probe hay contextos en los que no se puede dormir ni desmapear, y ahi NO se rescata.
 * Soltar la referencia con el CDMA parado y la cola vacia es exactamente el camino de cierre
 * normal (nvhost_channel_release usa nvhost_putchannel), asi que no se libera memoria en uso.
 * Se limita a 5 rescates por ventana de 30 s para no enmascarar un fallo de hardware permanente.
 *
 * BOTON, y esta vez con red de seguridad ✓: nace DESACTIVADO. Se enciende desde userspace con
 *   echo 1 > /sys/kernel/mocha_k76b/enable
 * o por linea de mando con mocha_k76b=1, que es como se dejara activo una vez medido que funciona.
 * Motivo, medido el 2026-11-21: la primera version del rescate colgo el arranque y hubo que volver
 * al bloque bueno por fastboot. Con el boton apagado por defecto, un fallo en el rescate no puede
 * impedir arrancar: el kernel se comporta como el K-76a (camera bloqueada hasta reiniciar, eso si).
 */
static int mocha_k76b;

/* true a partir de late_initcall: durante el arranque no se rescata nada. */
static bool mocha_pronto;

static ssize_t mocha_k76b_show(struct kobject *kobj, struct kobj_attribute *attr,
				   char *buf)
{
	return sprintf(buf, "%d\n", mocha_k76b);
}

static ssize_t mocha_k76b_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	int valor;

	if (kstrtoint(buf, 0, &valor))
		return -EINVAL;
	mocha_k76b = !!valor;
	pr_info("mocha K-76b: rescate de canal exclusivo %s desde /sys\n",
		mocha_k76b ? "activado" : "desactivado");
	return count;
}
static struct kobj_attribute mocha_k76b_attr_enable =
	__ATTR(enable, 0644, mocha_k76b_show, mocha_k76b_store);

static struct attribute *mocha_k76b_attrs[] = {
	&mocha_k76b_attr_enable.attr,
	NULL,
};

static struct attribute_group mocha_k76b_group = {
	.attrs = mocha_k76b_attrs,
};

static struct kobject *mocha_k76b_kobj;

static int __init mocha_k76b_late_init(void)
{
	mocha_k76b_kobj = kobject_create_and_add("mocha_k76b", kernel_kobj);
	if (mocha_k76b_kobj) {
		if (sysfs_create_group(mocha_k76b_kobj, &mocha_k76b_group)) {
			kobject_put(mocha_k76b_kobj);
			mocha_k76b_kobj = NULL;
		} else {
			pr_info("mocha K-76b: boton en /sys/kernel/mocha_k76b/enable "
				"(estado actual %d)\n", mocha_k76b);
		}
	} else {
		pr_info("mocha K-76b: no se pudo crear el boton /sys (estado actual %d)\n",
			mocha_k76b);
	}
	mocha_pronto = true;
	return 0;
}
late_initcall(mocha_k76b_late_init);

static int __init mocha_k76b_setup(char *s)
{
	mocha_k76b = simple_strtol(s, NULL, 0);
	pr_info("mocha K-76b: rescate de canal exclusivo %s por linea de mando\n",
		mocha_k76b ? "activado" : "desactivado");
	return 1;
}
__setup("mocha_k76b=", mocha_k76b_setup);

static atomic_t mocha_k76b_rescates = ATOMIC_INIT(0);
static unsigned long mocha_k76b_ventana;
static DEFINE_SPINLOCK(mocha_k76b_candado);

/* Devuelve true si queda cupo de rescate en la ventana en curso. */
static bool mocha_k76b_permitido(void)
{
	unsigned long flags;
	bool cupo = false;

	spin_lock_irqsave(&mocha_k76b_candado, flags);
	if (mocha_k76b_ventana == 0 || time_after(jiffies, mocha_k76b_ventana)) {
		mocha_k76b_ventana = jiffies + msecs_to_jiffies(30000);
		atomic_set(&mocha_k76b_rescates, 0);
	}
	if (atomic_inc_return(&mocha_k76b_rescates) <= 5)
		cupo = true;
	spin_unlock_irqrestore(&mocha_k76b_candado, flags);

	return cupo;
}

/*
 * true solo si el canal esta mapeado, sin dueno de contexto, CON EL DMA PARADO y la cola de
 * trabajos vacia. Ese es el estado 'referencia colgada sin trabajo vivo' que deja el
 * cdma_timeout_handler despues de 'nvhost_cdma_update_sync_queue'; solo ahi es seguro desmapear.
 * Si running sigue a true o queda algo en la cola, se devuelve false y se deja el -EBUSY tal cual:
 * preferimos la camara inoperativa hasta reiniciar antes que liberar un pushbuffer en uso.
 */
static bool mocha_canal_huerfano(struct nvhost_device_data *pdata,
				struct nvhost_channel *ch)
{
	const char *nombre;

	if (!mocha_pronto)
		return false;
	if (!ch || !ch->dev || ch->chid == NVHOST_INVALID_CHANNEL)
		return false;
	if (ch->refcount <= 0)
		return false;
	if (ch->cur_ctx)
		return false;

	/* Solo los modulos de camara (vi.0 / vi.1 / isp.0 / isp.1). */
	nombre = dev_name(&pdata->pdev->dev);
	if (strncmp(nombre, "vi", 2) != 0 && strncmp(nombre, "isp", 3) != 0)
		return false;

	/* El DMA tiene que estar parado y sin trabajos en vuelo: condicion de seguridad. */
	if (ch->cdma.running)
		return false;
	if (!list_empty(&ch->cdma.sync_queue))
		return false;

	return true;
}

/*
 * Suelta la referencia colgada. Debe llamarse SIN sujetar host->chlist_mutex, porque
 * nvhost_putchannel toma ese mismo mutex. No se hace ninguna espera: si el CDMA estuviera cargando
 * no habriamos llegado aqui (mocha_canal_huerfano lo impide).
 */
static void mocha_rescatar_canal(struct nvhost_device_data *pdata,
				struct nvhost_channel *ch)
{
	int ref_initial = ch->refcount;

	dev_err(&pdata->pdev->dev,
		"mocha K-76b: %s rescate del canal %d (chid %d) refcount %d cur_ctx %pK cdma: slots_used %u first_get 0x%x last_put 0x%x running %d torndown %d; DMA parado y cola vacia\n",
		dev_name(&pdata->pdev->dev), ch->dev_chid, ch->chid, ch->refcount, ch->cur_ctx,
		ch->cdma.slots_used, ch->cdma.first_get, ch->cdma.last_put, ch->cdma.running,
		ch->cdma.torndown);

	if (ch->refcount > 0) {
		nvhost_putchannel(ch, ch->refcount);
		dev_err(&pdata->pdev->dev,
			"mocha K-76b: %s liberadas %d referencia(s) colgadas; refcount ahora %d, canal desmapeado %s\n",
			dev_name(&pdata->pdev->dev), ref_initial, ch->refcount,
			ch->dev ? "NO" : "SI");
	}
}

/* Maps free channel with device */
int nvhost_channel_map(struct nvhost_device_data *pdata,
			struct nvhost_channel **channel)
{
	return nvhost_channel_map_interno(pdata, channel, 0);
}

static int nvhost_channel_map_interno(struct nvhost_device_data *pdata,
			struct nvhost_channel **channel, int ya_rescatado)
{
	struct nvhost_master *host = NULL;
	struct nvhost_channel *ch = NULL;
	int max_channels = 0;
	int index = 0;
	int err = 0;

	if (!pdata) {
		pr_err("%s: NULL device data\n", __func__);
		return -EINVAL;
	}

	host = nvhost_get_host(pdata->pdev);

	mutex_lock(&host->chlist_mutex);
	max_channels = host->info.nb_channels;

	/* Check if already channel(s) assigned for device */
	if (pdata->num_channels == pdata->num_mapped_chs) {
		if (pdata->exclusive) {
			struct nvhost_channel *huerfano = NULL;	/* mocha K-76b */
			/*
			 * mocha K-76a, medido 2026-11-20 en mocha (T124).  Este retorno era
			 * completamente silencioso, y es el origen de toda la cadena que dejaba la
			 * camara inservible hasta reiniciar la tableta:
			 *   'Opening channel /dev/nvhost-vi (0xc) failed'        (blob de 2015)
			 *   '__nvhost_channelopen: failed to map channel, -16'   (-EBUSY, aqui)
			 *   'camera3_device_open fallo con -22'                  (envoltorio)
			 *   'Camera3-Device: initialize: Could not open camera session: -22'
			 * En t124 los modulos de camara se declaran con .num_channels = 1 y
			 * .exclusive = true, asi que SOLO un cliente puede tener el canal mapeado.
			 * num_mapped_chs baja en nvhost_channel_unmap_locked, que solo se llama
			 * cuando nvhost_putchannel deja refcount en 0; host1x_channel_submit toma
			 * job->num_syncpts referencias por submit y las devuelve al completar el job
			 * (nvhost_intr_alloc_waiter / nvhost_putchannel).  Si el syncpoint del job no
			 * avanza nunca --lo vimos: 'cdma_timeout_handler: timeout: 22 (vi.0_2)' y el
			 * flood 'Waiting on the syncpt semaphore failed 0x2'-- la referencia queda
			 * colgada y el dispositivo exclusivo queda ciego para siempre.
			 * Imprimimos el estado para poder medirlo desde el log en vez de deducirlo.
			 */
			unsigned int mocha_ch;
			for (mocha_ch = 0; mocha_ch < (unsigned int)pdata->num_channels;
					mocha_ch++) {
				struct nvhost_channel *ocupado = pdata->channels[mocha_ch];
				if (!ocupado)
					continue;
				dev_err(&pdata->pdev->dev,
					"mocha K-76a: %s exclusivo ocupado; canal %u chid %d refcount %d cur_ctx %pK cdma: slots_used %u first_get 0x%x last_put 0x%x running %d torndown %d\n",
					dev_name(&pdata->pdev->dev), mocha_ch, ocupado->chid,
					ocupado->refcount, ocupado->cur_ctx,
					ocupado->cdma.slots_used, ocupado->cdma.first_get,
					ocupado->cdma.last_put, ocupado->cdma.running,
					ocupado->cdma.torndown);
				if (mocha_canal_huerfano(pdata, ocupado))
					huerfano = ocupado;
			}
			dev_err(&pdata->pdev->dev,
				"mocha K-76a: %s num_channels=%d num_mapped_chs=%d exclusive=%d keepalive=%d\n",
				dev_name(&pdata->pdev->dev), pdata->num_channels,
				pdata->num_mapped_chs, pdata->exclusive, pdata->keepalive);
			mutex_unlock(&host->chlist_mutex);

			/* mocha K-76b: si el dueno del canal esta muerto, se rescata y se reintenta. */
			if (huerfano && mocha_k76b && !ya_rescatado && mocha_k76b_permitido()) {
				int reintento;

				mocha_rescatar_canal(pdata, huerfano);
				reintento = nvhost_channel_map_interno(pdata, channel, 1);
				if (reintento == 0) {
					dev_info(&pdata->pdev->dev,
						"mocha K-76b: %s canal disponible tras el rescate\n",
						dev_name(&pdata->pdev->dev));
					return 0;
				}
				dev_err(&pdata->pdev->dev,
					"mocha K-76b: %s el reintento tras el rescate fallo (%d)\n",
					dev_name(&pdata->pdev->dev), reintento);
			}
			return -EBUSY;
		}
		ch = nvhost_check_channel(pdata);
		if (ch)
			ch->refcount++;
		mutex_unlock(&host->chlist_mutex);
		*channel = ch;
		return 0;
	}

	index = find_next_zero_bit(&host->allocated_channels,
					max_channels, host->next_free_ch);

	if (index >= max_channels) {
		/* Reset next pointer and try */
		host->next_free_ch = 0;
		index = find_next_zero_bit(&host->allocated_channels,
					max_channels, host->next_free_ch);
		if (index >= max_channels) {
			pr_err("All host1x channels are mapped, BITMAP: %lu\n",
					host->allocated_channels);
			mutex_unlock(&host->chlist_mutex);
			return -ENOMEM;
		}
	}

	/* Get channel from list and map to device */
	ch = host->chlist[index];
	if (!ch || (ch->chid != NVHOST_INVALID_CHANNEL)) {
		dev_err(&host->dev->dev, "%s: wrong channel map\n", __func__);
		mutex_unlock(&host->chlist_mutex);
		return -EINVAL;
	}

	ch->dev = pdata->pdev;
	ch->chid = index;
	nvhost_channel_assign(pdata, ch);
	nvhost_set_chanops(ch);
	set_bit(ch->chid, &host->allocated_channels);
	ch->refcount = 1;

	/* Initialize channel */
	err = nvhost_channel_init(ch, host);
	if (err) {
		dev_err(&ch->dev->dev, "%s: channel init failed\n", __func__);
		nvhost_channel_unmap_locked(ch);
		mutex_unlock(&host->chlist_mutex);
		return err;
	}

	/* set next free channel */
	if (index >= (max_channels - 1))
		host->next_free_ch = 0;
	else
		host->next_free_ch = index + 1;

	if (pdata->init && pdata->num_mapped_chs == 1) {
		err = pdata->init(ch->dev);
		if (err) {
			dev_err(&ch->dev->dev, "device init failed\n");
			nvhost_channel_unmap_locked(ch);
			mutex_unlock(&host->chlist_mutex);
			return err;
		}
	}

	/* Keep alive modules that needs to be when a channel is open */
	if (pdata->keepalive && pdata->num_mapped_chs)
		nvhost_module_disable_poweroff(pdata->pdev);

	dev_dbg(&ch->dev->dev, "channel %d mapped\n", ch->chid);
	mutex_unlock(&host->chlist_mutex);

	*channel = ch;
	return 0;
}

/* Free channel memory and list */
int nvhost_channel_list_free(struct nvhost_master *host)
{
	int i;

	for (i = 0; i < host->info.nb_channels; i++)
		kfree(host->chlist[i]);

	dev_info(&host->dev->dev, "channel list free'd\n");

	return 0;
}

int nvhost_channel_init(struct nvhost_channel *ch,
		struct nvhost_master *dev)
{
	int err;

	/* Link platform_device to nvhost_channel */
	err = channel_op(ch).init(ch, dev);
	if (err < 0) {
		dev_err(&dev->dev->dev, "failed to init channel %d\n",
				ch->chid);
		return err;
	}

	return nvhost_cdma_init(&ch->cdma);
}

void nvhost_channel_init_gather_filter(struct nvhost_channel *ch)
{
	if (channel_op(ch).init_gather_filter)
		channel_op(ch).init_gather_filter(ch);
}

int nvhost_channel_submit(struct nvhost_job *job)
{
	/*
	 * Check if queue has higher priority jobs running. If so, wait until
	 * queue is empty. Ignores result from nvhost_cdma_flush, as we submit
	 * either when push buffer is empty or when we reach the timeout.
	 */
	int higher_count = 0;

	switch (job->priority) {
	case NVHOST_PRIORITY_HIGH:
		higher_count = 0;
		break;
	case NVHOST_PRIORITY_MEDIUM:
		higher_count = job->ch->cdma.high_prio_count;
		break;
	case NVHOST_PRIORITY_LOW:
		higher_count = job->ch->cdma.high_prio_count
			+ job->ch->cdma.med_prio_count;
		break;
	}
	if (higher_count > 0)
		(void)nvhost_cdma_flush(&job->ch->cdma,
				NVHOST_CHANNEL_LOW_PRIO_MAX_WAIT);

	return channel_op(job->ch).submit(job);
}


void nvhost_getchannel(struct nvhost_channel *ch)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(ch->dev);
	struct nvhost_master *host = nvhost_get_host(pdata->pdev);

	mutex_lock(&host->chlist_mutex);
	ch->refcount++;
	mutex_unlock(&host->chlist_mutex);
}

void nvhost_putchannel(struct nvhost_channel *ch, int cnt)
{
	struct nvhost_device_data *pdata;
	struct nvhost_master *host;

	/*
	 * mocha K-76b: defensivo. Tras un rescate del canal exclusivo el trabajo que expiro puede
	 * completar despues y volver a llamar aqui sobre un canal ya desmapeado; en ese caso
	 * ch->dev es NULL y platform_get_drvdata() habria seguido un puntero nulo. Se registra y se
	 * sale sin tocar nada, que es lo que hacia el codigo original cuando el canal ya estaba
	 * libre (nvhost_channel_unmap_locked imprimia 'freeing unmapped channel').
	 */
	if (!ch || !ch->dev) {
		pr_warn("mocha K-76b: nvhost_putchannel sobre canal sin dispositivo, ignorado\n");
		return;
	}

	pdata = platform_get_drvdata(ch->dev);
	host = nvhost_get_host(pdata->pdev);

	mutex_lock(&host->chlist_mutex);
	ch->refcount -= cnt;

	/* WARN on negative reference, with zero reference unmap channel*/
	if (!ch->refcount)
		nvhost_channel_unmap_locked(ch);
	else if (ch->refcount < 0) {
		/* mocha K-76b: se acota a cero en vez de dejar el contador negativo y enmudecer. */
		dev_warn(&ch->dev->dev,
			"mocha K-76b: refcount negativo (%d) en canal %d; se fija a 0\n",
			ch->refcount, ch->chid);
		ch->refcount = 0;
		nvhost_channel_unmap_locked(ch);
	}
	mutex_unlock(&host->chlist_mutex);
}

int nvhost_channel_suspend(struct nvhost_channel *ch)
{
	int ret = 0;

	if (channel_cdma_op().stop && ch->dev)
		channel_cdma_op().stop(&ch->cdma);

	return ret;
}

struct nvhost_channel *nvhost_alloc_channel_internal(int chindex,
			int max_channels)
{
	struct nvhost_channel *ch = NULL;

	ch = kzalloc(sizeof(*ch), GFP_KERNEL);
	if (ch)
		ch->chid = chindex;

	return ch;
}

int nvhost_channel_save_context(struct nvhost_channel *ch)
{
	int err = 0;

	if (ch && ch->cur_ctx)
		err = channel_op(ch).save_context(ch);

	return err;

}

static struct nvhost_hwctx *alloc_hwctx(struct nvhost_hwctx_handler *h,
		struct nvhost_channel *ch)
{
	struct nvhost_hwctx *ctx;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return NULL;

	kref_init(&ctx->ref);
	ctx->h = h;
	ctx->channel = ch;
	ctx->valid = true;

	return ctx;
}

static void free_hwctx(struct kref *ref)
{
	struct nvhost_hwctx *ctx = container_of(ref, struct nvhost_hwctx, ref);

	kfree(ctx);
}

static void get_hwctx(struct nvhost_hwctx *ctx)
{
	kref_get(&ctx->ref);
}

static void put_hwctx(struct nvhost_hwctx *ctx)
{
	kref_put(&ctx->ref, free_hwctx);
}

struct nvhost_hwctx_handler *nvhost_alloc_hwctx_handler(u32 syncpt,
	u32 waitbase, struct nvhost_channel *ch)
{
	struct nvhost_hwctx_handler *p;

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return NULL;

	p->syncpt = NVSYNCPT_INVALID;
	p->waitbase = waitbase;

	p->alloc = alloc_hwctx;
	p->get   = get_hwctx;
	p->put   = put_hwctx;

	return p;
}
