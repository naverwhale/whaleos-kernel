// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2011-2020, The Linux Foundation. All rights reserved.
 */

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/io-64-nonatomic-hi-lo.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>

#include <linux/soc/qcom/smem.h>
#include <clocksource/arm_arch_timer.h>

#define STAT_TYPE_ADDR		0x0
#define COUNT_ADDR		0x4
#define LAST_ENTERED_AT_ADDR	0x8
#define LAST_EXITED_AT_ADDR	0x10
#define ACCUMULATED_ADDR	0x18
#define CLIENT_VOTES_ADDR	0x1c

struct subsystem_data {
	const char *name;
	u32 smem_item;
	u32 pid;
};

static struct subsystem_data subsystems[] = {
	{ "modem", 605, 1 },
	{ "adsp", 606, 2 },
	{ "cdsp", 607, 5 },
	{ "slpi", 608, 3 },
	{ "gpu", 609, 0 },
	{ "display", 610, 0 },
};

struct stats_config {
	unsigned int offset_addr;
	unsigned int num_records;
	bool appended_stats_avail;
};

struct stats_prv_data {
	const struct stats_config *config;
	void __iomem *reg;
};

struct sleep_stats {
	u32 stat_type;
	u32 count;
	u64 last_entered_at;
	u64 last_exited_at;
	u64 accumulated;
};

struct appended_stats {
	u32 client_votes;
	u32 reserved[3];
};

static void print_sleep_stats(struct seq_file *s, struct sleep_stats *stat)
{
	u64 accumulated = stat->accumulated;
	/*
	 * If a subsystem is in sleep when reading the sleep stats adjust
	 * the accumulated sleep duration to show actual sleep time.
	 */
	if (stat->last_entered_at > stat->last_exited_at)
		accumulated += arch_timer_read_counter()
			       - stat->last_entered_at;

	seq_printf(s, "Count = %u\n", stat->count);
	seq_printf(s, "Last Entered At = %llu\n", stat->last_entered_at);
	seq_printf(s, "Last Exited At = %llu\n", stat->last_exited_at);
	seq_printf(s, "Accumulated Duration = %llu\n", accumulated);
}

static int subsystem_sleep_stats_show(struct seq_file *s, void *d)
{
	struct subsystem_data *subsystem = s->private;
	struct sleep_stats *stat;

	stat = qcom_smem_get(subsystem->pid, subsystem->smem_item, NULL);
	if (IS_ERR(stat))
		return PTR_ERR(stat);

	print_sleep_stats(s, stat);

	return 0;
}

static int soc_sleep_stats_show(struct seq_file *s, void *d)
{
	struct stats_prv_data *prv_data = s->private;
	void __iomem *reg = prv_data->reg;
	struct sleep_stats stat;

	stat.count = readl(reg + COUNT_ADDR);
	stat.last_entered_at = readq(reg + LAST_ENTERED_AT_ADDR);
	stat.last_exited_at = readq(reg + LAST_EXITED_AT_ADDR);
	stat.accumulated = readq(reg + ACCUMULATED_ADDR);

	print_sleep_stats(s, &stat);

	if (prv_data->config->appended_stats_avail) {
		struct appended_stats app_stat;

		app_stat.client_votes = readl(reg + CLIENT_VOTES_ADDR);
		seq_printf(s, "Client_votes = %#x\n", app_stat.client_votes);
	}

	return 0;
}

DEFINE_SHOW_ATTRIBUTE(soc_sleep_stats);
DEFINE_SHOW_ATTRIBUTE(subsystem_sleep_stats);

static struct dentry *create_debugfs_entries(void __iomem *reg,
					     struct stats_prv_data *prv_data)
{
	struct dentry *root;
	struct sleep_stats *stat;
	char stat_type[sizeof(u32) + 1] = {0};
	u32 offset, type;
	int i;

	root = debugfs_create_dir("qcom_sleep_stats", NULL);

	for (i = 0; i < prv_data[0].config->num_records; i++) {
		offset = STAT_TYPE_ADDR + (i * sizeof(struct sleep_stats));

		if (prv_data[0].config->appended_stats_avail)
			offset += i * sizeof(struct appended_stats);

		prv_data[i].reg = reg + offset;

		type = readl(prv_data[i].reg);
		memcpy(stat_type, &type, sizeof(u32));
		debugfs_create_file(stat_type, 0444, root,
				    &prv_data[i],
				    &soc_sleep_stats_fops);
	}

	for (i = 0; i < ARRAY_SIZE(subsystems); i++) {
		stat = qcom_smem_get(subsystems[i].pid, subsystems[i].smem_item,
				     NULL);
		if (IS_ERR(stat))
			continue;

		debugfs_create_file(subsystems[i].name, 0444, root,
				    &subsystems[i],
				    &subsystem_sleep_stats_fops);
	}

	return root;
}

static int soc_sleep_stats_probe(struct platform_device *pdev)
{
	struct resource *res;
	void __iomem *reg;
	void __iomem *offset_addr;
	phys_addr_t stats_base;
	resource_size_t stats_size;
	struct dentry *root;
	const struct stats_config *config;
	struct stats_prv_data *prv_data;
	int i;

	config = device_get_match_data(&pdev->dev);
	if (!config)
		return -ENODEV;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return PTR_ERR(res);

	offset_addr = ioremap(res->start + config->offset_addr, sizeof(u32));
	if (IS_ERR(offset_addr))
		return PTR_ERR(offset_addr);

	stats_base = res->start | readl_relaxed(offset_addr);
	stats_size = resource_size(res);
	iounmap(offset_addr);

	reg = devm_ioremap(&pdev->dev, stats_base, stats_size);
	if (!reg)
		return -ENOMEM;

	prv_data = devm_kzalloc(&pdev->dev, config->num_records *
				sizeof(struct stats_prv_data), GFP_KERNEL);
	if (!prv_data)
		return -ENOMEM;

	for (i = 0; i < config->num_records; i++)
		prv_data[i].config = config;

	root = create_debugfs_entries(reg, prv_data);
	platform_set_drvdata(pdev, root);

	return 0;
}

static int soc_sleep_stats_remove(struct platform_device *pdev)
{
	struct dentry *root = platform_get_drvdata(pdev);

	debugfs_remove_recursive(root);

	return 0;
}

static const struct stats_config rpm_data = {
	.offset_addr = 0x14,
	.num_records = 2,
	.appended_stats_avail = true,
};

static const struct stats_config rpmh_data = {
	.offset_addr = 0x4,
	.num_records = 3,
	.appended_stats_avail = false,
};

static const struct of_device_id soc_sleep_stats_table[] = {
	{ .compatible = "qcom,rpm-sleep-stats", .data = &rpm_data },
	{ .compatible = "qcom,rpmh-sleep-stats", .data = &rpmh_data },
	{ }
};

static struct platform_driver soc_sleep_stats_driver = {
	.probe = soc_sleep_stats_probe,
	.remove = soc_sleep_stats_remove,
	.driver = {
		.name = "soc_sleep_stats",
		.of_match_table = soc_sleep_stats_table,
	},
};
module_platform_driver(soc_sleep_stats_driver);

MODULE_DESCRIPTION("Qualcomm Technologies, Inc. (QTI) SoC Sleep Stats driver");
MODULE_LICENSE("GPL v2");
