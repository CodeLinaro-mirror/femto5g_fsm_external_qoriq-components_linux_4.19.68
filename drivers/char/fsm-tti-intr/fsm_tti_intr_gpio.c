/* Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/of.h>
#include <linux/mm.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>

#include "fsm_tti_intr.h"

static irqreturn_t fsm_tti_gpio_irq_handler(int irq, void *irq_data)
{
	struct fsm_tti_mmap_info *sdata;
	struct fsm_tti_intr_drv *tti_intr_drv =
		(struct fsm_tti_intr_drv *)irq_data;

	/* update the SFN and slot number */
	sdata = tti_intr_drv->shared_data;
	if (sdata && (tti_intr_drv->is_seeding_done)) {
		/* Update stats */
		sdata->abs_recv_time = ktime_get();
		tti_intr_drv->debugfs_stats.current_tti_recv_time =
			sdata->abs_recv_time;

		/* Keep track of the time when first tti intr is received */
		if (tti_intr_drv->is_first_tti_intr == false) {
			tti_intr_drv->is_first_tti_intr = true;
			tti_intr_drv->debugfs_stats.first_tti_recv_time =
				sdata->abs_recv_time;
		} else {

			sdata->sfn_slot_info.slot =
				(sdata->sfn_slot_info.slot + 1) %
				sdata->max_slot;

			if (sdata->sfn_slot_info.slot == 0)
				sdata->sfn_slot_info.sfn =
				(sdata->sfn_slot_info.sfn + 1) &
				FSM_TTI_MAX_SFN_MOD_FACTOR;

			/* Make sure sfn/slot is updated before moving ahead */
			smp_mb();
		}
		sdata->intr_recv_count = sdata->intr_recv_count + 1;
		tti_intr_drv->debugfs_stats.current_tti_count =
			sdata->intr_recv_count;
		/* Make sure timestamps are updated before sfn/slot */
		smp_mb();

		/* wake up the poll ops */
		if (tti_intr_drv->is_poll_enabled) {
			atomic_set(&tti_intr_drv->tti_updated, 1);
			wake_up(&tti_intr_drv->tti_poll_waitqueue);
		}
	}
	return IRQ_HANDLED;
}

static int __init fsm_tti_intr_probe(struct platform_device *pdev)
{
	int ret;
	struct page *page;
	unsigned long flags;
	struct device_node *np;
	struct fsm_tti_intr_drv *tti_intr_drv;
	struct fsm_tti_gpio_device_data *device_data;
	struct fsm_tti_intr_drv *p;
	bool assert_falling_edge = false;
	int gpio_pin = -1;
	int num_fsm = 0;
	struct fsm_tti_mmap_info *sdata;
	char gpio_label[256];
	int i;

	FSM_TTI_INFO("FSM-TTI: probing device\n");

	tti_intr_drv = kzalloc(sizeof(*tti_intr_drv) * MAX_FSM_TTI_DEVICE,
					GFP_KERNEL);
	if (IS_ERR(tti_intr_drv))
		return -ENOMEM;
	np = pdev->dev.of_node;
	if (!np) {
		kfree(tti_intr_drv);
		return -ENOENT;
	}
	/* allocate space for device info */
	device_data = devm_kzalloc(&pdev->dev, MAX_FSM_TTI_DEVICE *
			sizeof(struct fsm_tti_gpio_device_data), GFP_KERNEL);
	if (!device_data) {
		kfree(tti_intr_drv);
		return -ENOMEM;
	}
	if (of_get_property(np, "assert-falling-edge", NULL))
		assert_falling_edge = true;

	for (i = 0, p = tti_intr_drv; i < MAX_FSM_TTI_DEVICE; i++, p++) {
		p->dev = &pdev->dev;
		p->device_data =  device_data + i;
		/* read DTS to determine number of FSM */
		gpio_pin = of_get_gpio(np, i);
		if (gpio_pin < 0)
			goto probe_cont;
		p->device_data->gpio_pin = gpio_pin;
		if (i == 0)
			strlcpy(gpio_label, FSM_TTI_GPIO_NAME,
				sizeof(gpio_label));
		else
			snprintf(gpio_label, sizeof(gpio_label),
				"%s_%d", FSM_TTI_GPIO_NAME, i + 1);
		p->device_data->assert_falling_edge = assert_falling_edge;
		/* GPIO setup */
		ret = devm_gpio_request(&pdev->dev,
				p->device_data->gpio_pin,
				gpio_label);
		if (ret) {
			dev_err(&pdev->dev, "failed to request GPIO %u\n",
				p->device_data->gpio_pin);
			goto cleanup_shared_data;
		}
		ret = gpio_direction_input(p->device_data->gpio_pin);
		if (ret) {
			dev_err(&pdev->dev, "failed to set pin direction\n");
			goto cleanup_shared_data;
		}
		ret = gpio_to_irq(p->device_data->gpio_pin);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to map GPIO to IRQ: %d\n",
					ret);
			goto cleanup_shared_data;
		}
		p->device_data->irq = ret;
		flags = p->device_data->assert_falling_edge ?
			IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
		if (p->device_data->capture_clear) {
			flags |= ((flags & IRQF_TRIGGER_RISING) ?
				IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING);
		}
		if (i == 0)
			snprintf(p->device_data->name,
				FSM_TTI_MAX_NAME_LEN - 1,
				"%s.%d",
				pdev->name, pdev->id);
		else
			snprintf(p->device_data->name,
				FSM_TTI_MAX_NAME_LEN - 1,
				"%s.%d_%d",
				pdev->name, pdev->id, i + 1);
		ret = devm_request_irq(&pdev->dev,
				p->device_data->irq,
				fsm_tti_gpio_irq_handler,
				flags,
				p->device_data->name,
				p);
		if (ret) {
			dev_err(&pdev->dev, "failed to acquire IRQ %d\n",
			tti_intr_drv->device_data->irq);
			goto cleanup_shared_data;
		}
		page = alloc_page(GFP_KERNEL);
		if (page) {
			p->page = page;
			p->shared_data =
				(struct fsm_tti_mmap_info *)page_address(page);
			sdata = p->shared_data;
			sdata->sfn_slot_info.sfn_slot = 0xdeadbeef;
		} else
			goto cleanup_shared_data;
		/* initialize wait queue */
		init_waitqueue_head(&p->tti_poll_waitqueue);
		/* initialize the flags */
		atomic_set(&p->tti_updated, 0);
		p->is_seeding_done = false;
		p->is_poll_enabled = false;
		p->is_first_tti_intr = false;
		num_fsm++;
	}
probe_cont:
	/* keep a driver reference to the device structure */
	if (!num_fsm)
		goto cleanup;
	platform_set_drvdata(pdev, tti_intr_drv);

	for (i = 0, p = tti_intr_drv; i < num_fsm; i++, p++)
		p->num_fsm = num_fsm;

	/* initialize char interface to userspace */
	ret = fsm_tti_cdev_init(tti_intr_drv);
	if (ret)
		goto cleanup_shared_data;

	ret = fsm_tti_debugfs_init(tti_intr_drv);
	if (ret)
		goto cleanup_cdev;


	FSM_TTI_INFO("FSM-TTI: module initialized\n");
	return 0;

cleanup_cdev:
	fsm_tti_cdev_cleanup(tti_intr_drv);
cleanup_shared_data:
	for (i = 0, p = tti_intr_drv; i < num_fsm; i++, p++)
		if (p->page) {
			__free_page(p->page);
		p->page = NULL;
		p->shared_data = NULL;
		p->shared_data = NULL;
	}
cleanup:
	devm_kfree(&pdev->dev, device_data);
	kfree(tti_intr_drv);
	FSM_TTI_ERROR("FSM-TTI: module init failed!\n");
	return ret;
}

static int __exit fsm_tti_intr_remove(struct platform_device *pdev)
{
	struct fsm_tti_intr_drv *tti_intr_drv = platform_get_drvdata(pdev);
	struct fsm_tti_intr_drv *p;
	int i;

	if (tti_intr_drv) {
		fsm_tti_debugfs_cleanup(tti_intr_drv);
		fsm_tti_cdev_cleanup(tti_intr_drv);
		for (i = 0, p = tti_intr_drv; i < tti_intr_drv->num_fsm;
						i++, p++) {
			if (p->page)
				__free_page(p->page);
			p->page = NULL;
			p->shared_data = NULL;
		}
		devm_kfree(&pdev->dev, tti_intr_drv->device_data);
		kfree(tti_intr_drv);
	}
	FSM_TTI_INFO("FSM-TTI: module removed\n");
	return 0;
}

static const struct of_device_id fsm_tti_intr_of_table[] = {
	{ .compatible = "tti-gpio" },
	{ },
};
MODULE_DEVICE_TABLE(of, fsm_tti_intr_of_table);

static struct platform_driver __fsm_tti_intr_platform_drv = {
	.probe  = fsm_tti_intr_probe,
	.remove = fsm_tti_intr_remove,
	.driver = {
		.name           = KBUILD_MODNAME,
		.of_match_table = fsm_tti_intr_of_table,
		.owner          = THIS_MODULE,
	},
};

module_platform_driver(__fsm_tti_intr_platform_drv);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("FSM TTI interrupt driver");

