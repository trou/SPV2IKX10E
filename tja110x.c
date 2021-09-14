/*
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/phy.h>
#include <linux/netdevice.h>
#include <linux/of_gpio.h>

/**
 * Enable polling infrastructure, that regularly reads the interrupt status
 * register and handles certain interrupts.
 * If not defined, the GPIO specified in the device tree will be used as
 * interrupt input, and polling will not be enabled.
 */
/* #define CONFIG_POLL */

#include "tja110x.h"

/* load driver for TJA1102p1. It needs to be ensured,
 * that no other mdio device with phy id 0 is present
 */
#define CONFIG_TJA1102_FIX

/* Further configuration that only applies if polling is used */
#ifdef CONFIG_POLL
	/* Initialize phy during probing and
	 * listen for NETDEV_GOING_DOWN and NETDEV_UP events of the ethernet
	 * interface controlling the mdio bus connected to the phy.
	 * Polling is stopped/started accordingly, to prevent mdio read timeouts
	 *
	 * This setting requires MDIO_INTERFACE_NAME and MII_BUS_NAME to be set
	 * and allows using the phy without an attached netdev.
	 */
	/* #define CONFIG_STANDALONE_PHY */

	/* Name of the eth interface, that controlls the mdio bus,
	 * to which the phy(s) is/are connected to
	 */
	#ifndef MDIO_INTERFACE_NAME
	#define MDIO_INTERFACE_NAME "eth0"
	#endif

	/* Name of the mdio bus,
	 * to which the phy(s) is/are connected to
	 */
	#ifndef MII_BUS_NAME
	#define MII_BUS_NAME "fec_enet_mii_bus"
	#endif
#endif


/* Variable can be modified via parameter passed at load time
 * A nonzero value indicates that we should operate in managed mode
 */
static int managed_mode;
/* Permission: do not show up in sysfs */
module_param(managed_mode, int, 0000);
MODULE_PARM_DESC(managed_mode, "Use PHY in managed or autonomous mode");

/* Determines the level of verbosity for debug messages */
static int verbosity;
/* Permission: do not show up in sysfs */
module_param(verbosity, int, 0000);
MODULE_PARM_DESC(verbosity, "Set verbosity level");

#ifdef CONFIG_POLL
/* A nonzero value indicates that we should not poll the interrupt register */
static int no_poll;
/* Permission: do not show up in sysfs */
module_param(no_poll, int, 0000);
MODULE_PARM_DESC(no_poll, "Do not poll the interrupt register");
#else
static const int no_poll = 1;
#endif

static struct attribute_group nxp_attribute_group;

#ifndef CONFIG_POLL

/* Retrieve GPIO that is used for interrupts from the device tree */
static int get_and_request_gpio(struct phy_device *phydev)
{
	int gpio_num;
	struct device_node *node;

	node = phydev->PHYDEV_DEV.of_node;
	if (!node)
		goto error;

	gpio_num = of_get_named_gpio(node, "irq-gpio", 0);
	if (!gpio_is_valid(gpio_num))
		goto error;

	if (gpio_request(gpio_num, "phy_interrupt") < 0)
		if (verbosity > 0)
			dev_err(&phydev->PHYDEV_DEV,
				"GPIO request failed (already requested?)\n");

	return gpio_num;

/* error handling */
error:
	dev_err(&phydev->PHYDEV_DEV, "Could not retrieve GPIO from device tree\n");
	return -ENODEV;
}
#endif

/* Called to initialize the PHY,
 * including after a reset
 */
static int nxp_config_init(struct phy_device *phydev)
{
	int reg_val;
	int reg_name, reg_value = -1, reg_mask;
	int err;

	if (verbosity > 0)
		dev_alert(&phydev->PHYDEV_DEV, "initializing phy %x\n", phydev->PHYDEV_ADDR);

	/* set features of the PHY */
	phydev->supported   = 0;
	phydev->advertising = 0;
	phydev->autoneg     = AUTONEG_DISABLE;

	reg_val = phy_read(phydev, MII_BMSR);
	if (reg_val < 0)
		goto phy_read_error;
	if (reg_val & BMSR_ESTATEN) {
		reg_val = phy_read(phydev, MII_ESTATUS);

		if (reg_val < 0)
			goto phy_read_error;

		if (reg_val & ESTATUS_100T1_FULL) {
			/* update phydev to include the supported features */
			phydev->supported |= SUPPORTED_100BASET1_FULL;
		}
	}

	/* enable configuration register access once during initialization */
	err = phy_configure_bit(phydev, MII_ECTRL, ECTRL_CONFIG_EN, 1);
	if (err < 0)
		goto phy_configure_error;

	/* -enter managed or autonomous mode,
	 *  depending on the value of managed_mode.
	 *  The register layout changed between TJA1100 and TJA1102
	 * -configure LED mode (only tja1100 has LEDs)
	 */
	switch (phydev->phy_id & NXP_PHY_ID_MASK) {
	case NXP_PHY_ID_TJA1100:
		reg_name = MII_CFG1;
		reg_value = TJA1100_CFG1_LED_EN | CFG1_LED_LINKUP;
		if (!managed_mode)
			reg_value |= TJA1100_CFG1_AUTO_OP;
		reg_mask = TJA1100_CFG1_AUTO_OP |
		    TJA1100_CFG1_LED_EN | TJA1100_CFG1_LED_MODE;
		break;
	case NXP_PHY_ID_TJA1101:
		/* fall through */
	case NXP_PHY_ID_TJA1102P0:
		reg_name = MII_COMMCFG;
		reg_value = 0;
		if (!managed_mode)
			reg_value |= COMMCFG_AUTO_OP;
		reg_mask = COMMCFG_AUTO_OP;
		break;

	case NXP_PHY_ID_TJA1102P1:
		/* does not have an auto_op bit */
		break;

	default:
		goto unsupported_phy_error;
	}

	/* only configure the phys that have an auto_op bit or LEDs */
	if (reg_value != -1) {
		err = phy_configure_bits(phydev, reg_name, reg_mask, reg_value);
		if (err < 0)
			goto phy_configure_error;
	}

	/* enable sleep confirm */
	err = phy_configure_bit(phydev, MII_CFG1, CFG1_SLEEP_CONFIRM, 1);
	if (err < 0)
		goto phy_configure_error;

	/* set sleep request timeout to 16ms */
	err = phy_configure_bits(phydev, MII_CFG2, CFG2_SLEEP_REQUEST_TO,
				 SLEEP_REQUEST_TO_16MS);
	if (err < 0)
		goto phy_configure_error;

	/* if in managed mode:
	 * -go to normal mode, if currently in standby
	 * (PHY might be pinstrapped to managed mode,
	 * and therefore not in normal mode yet)
	 * -enable link control
	 */
	if (managed_mode) {
		reg_val = phy_read(phydev, MII_ECTRL);
		if (reg_val < 0)
			goto phy_read_error;

		/* mask power mode bits */
		reg_val &= ECTRL_POWER_MODE;

		if (reg_val == POWER_MODE_STANDBY) {
			err = phydev->drv->resume(phydev);
			if (err < 0)
				goto phy_pmode_transit_error;
		}

		set_link_control(phydev, 1);
	}

#ifdef CONFIG_POLL
	/* clear any pending interrupts */
	phydev->drv->ack_interrupt(phydev);

	phydev->irq = PHY_POLL;

	/* enable all interrupts */
	phydev->interrupts = PHY_INTERRUPT_ENABLED;
	phydev->drv->config_intr(phydev);

	/* Setup and queue a polling function */
	if (!no_poll) {
		setup_polling(phydev);
		start_polling(phydev);
	}
#else
	/* Set trigger mode to falling edge
	 *
	 * This configuration is reset when phy_stop_interrupts is called.
	 * It needs to be reconfigured every time the phy is initialized.
	 */
	if (phydev->irq > 0)
		irq_set_irq_type(phydev->irq, IRQ_TYPE_EDGE_FALLING);
#endif

	return 0;

/* error handling */
phy_read_error:
	dev_err(&phydev->PHYDEV_DEV, "read error: %s failed\n", __func__);
	return reg_val;

phy_pmode_transit_error:
	dev_err(&phydev->PHYDEV_DEV, "pmode error: %s failed\n", __func__);
	return err;

phy_configure_error:
	dev_err(&phydev->PHYDEV_DEV, "read/write error: %s failed\n", __func__);
	return err;

unsupported_phy_error:
	dev_err(&phydev->PHYDEV_DEV, "unsupported phy, %s failed\n", __func__);
	return -1;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,1)) && defined(CONFIG_STANDALONE_PHY)
/**
 * Starting with 3c293f4e08b58ad5b78f78d89ca1fd41f87f8729 (v4.9-rc1) the
 * state machine is triggered when phy_start() is called.
 * mdio_netdev_change_event() calls phy_start().
 *
 * Starting with a81497bee70eb15039594b3116913133aa9c9b29 (v4.14-rc1)
 * there is a new callback (phy_link_change), that is called from within the
 * state machine. The callback is usually initialized in phy_attach_direct(),
 * in standalone mode it needs to be set up manually.
 */
void nxp_link_change(struct phy_device *phydev, bool up, bool do_carrier)
{
	/* stub */
}
#endif

/* Called during discovery.
 * Used to set up device-specific structures
 */
static int nxp_probe(struct phy_device *phydev)
{
	int err;
	nxp_specific_data_t *nxp_specific;
#ifndef CONFIG_POLL
	int gpio, irq;
#endif

	if (verbosity > 0)
		dev_alert(&phydev->PHYDEV_DEV, "probing PHY %x\n", phydev->PHYDEV_ADDR);

	nxp_specific = kzalloc(sizeof(*nxp_specific), GFP_KERNEL);
	if (!nxp_specific)
		goto phy_allocation_error;

	nxp_specific->is_master = get_master_cfg(phydev);

#ifdef CONFIG_POLL
	nxp_specific->is_polling = false;
	nxp_specific->is_poll_setup = false;
#else
	/* get a valid irq from dts */
	err = 0;
	gpio = get_and_request_gpio(phydev);
	if (gpio < 0) {
		err = 1;
		nxp_specific->gpio = -ENODEV;
	} else {
		nxp_specific->gpio = gpio;
		irq = gpio_to_irq(nxp_specific->gpio);
		if (irq < 0)
			err = 1;
		else
			phydev->irq = irq;
	}

	if (err) {
		dev_err(&phydev->PHYDEV_DEV, "Ignoring interrupts\n");
		phydev->irq = PHY_IGNORE_INTERRUPT;
	}
#endif

	phydev->priv = nxp_specific;

#if defined(CONFIG_STANDALONE_PHY) && defined(CONFIG_POLL)

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,1)
	phydev->phy_link_change = nxp_link_change;
#endif

	/* Initialize the phy in case it should be operated in standalone
	 * mode. config_init would not be called otherwise
	 */
	err = phydev->drv->config_init(phydev);
	if (err < 0)
		return err;
#endif

	/* register sysfs files */
	err = sysfs_create_group(&phydev->PHYDEV_DEV.kobj, &nxp_attribute_group);
	if (err)
		goto register_sysfs_error;

	return 0;

/* error handling */
register_sysfs_error:
	dev_err(&phydev->PHYDEV_DEV, "sysfs file creation failed\n");
	return -ENOMEM;

phy_allocation_error:
	dev_err(&phydev->PHYDEV_DEV, "memory allocation for priv data failed\n");
	return -ENOMEM;
}

/* Clears up any memory, removes sysfs nodes and cancels polling */
static void nxp_remove(struct phy_device *phydev)
{
	if (verbosity > 0)
		dev_alert(&phydev->PHYDEV_DEV, "removing PHY %x\n", phydev->PHYDEV_ADDR);

	/* unregister sysfs files */
	sysfs_remove_group(&phydev->PHYDEV_DEV.kobj, &nxp_attribute_group);

	/* disconnect from any attached devices, and stop the state machine */
	if (phydev->attached_dev && phydev->adjust_link)
		phy_disconnect(phydev);

#ifdef CONFIG_POLL
	if (!no_poll)
		stop_polling(phydev);
#else
	/* Free the GPIO pin */
	if (PHY_PRIV(phydev)->gpio >= 0)
		gpio_free(PHY_PRIV(phydev)->gpio);
#endif

	/* free custom data struct */
	if (phydev->priv) {
		kzfree(phydev->priv);
		phydev->priv = NULL;
	}
}

/* check the link status and update current link state */
static int nxp_read_status(struct phy_device *phydev)
{
	int link_status;

	/* Update the link, but return if there was an error */
	link_status = get_link_status(phydev);
	if (link_status < 0)
		return link_status;
	phydev->link = !!link_status;

	phydev->speed = SPEED_100;
	phydev->duplex = DUPLEX_FULL;
	phydev->pause = 0;
	phydev->asym_pause = 0;

	return 0;
}

/* Clears any pending interrupts */
static int nxp_ack_interrupt(struct phy_device *phydev)
{
	int err;

	if (verbosity > 3)
		dev_alert(&phydev->PHYDEV_DEV, "acknowledging interrupt of PHY %x\n",
		phydev->PHYDEV_ADDR);

	/* interrupts are acknowledged by reading, ie. clearing MII_INTSRC */
	err = phy_read(phydev, MII_INTSRC);
	if (err < 0)
		goto phy_read_error;
	return 0;

/* error handling */
phy_read_error:
	dev_err(&phydev->PHYDEV_DEV, "read error: %s failed\n", __func__);
	return err;
}

/* Called whenever there was an interrupt on the phys IRQ line,
 * checks if phy generated the interrupt.
 */
static int nxp_did_interrupt(struct phy_device *phydev)
{
	int intr = handle_interrupts(phydev);

	/* If the returned bitmask includes any set interrupts, the phydev
	 * framework will check phydev->state, and set it to PHY_CHANGELINK
	 * if needed. If the state machine is running,
	 * this will cause phydev->adjust_link to be called.
	 */
	return intr & (INTERRUPT_LINK_STATUS_FAIL | INTERRUPT_LINK_STATUS_UP);
}

/* Enables or disables interrupts */
static int nxp_config_intr(struct phy_device *phydev)
{
	int err;
	int interrupts;

	if (verbosity > 0)
		dev_alert(&phydev->PHYDEV_DEV,
		"configuring interrupts of phy %x to [%x]\n",
		phydev->PHYDEV_ADDR, phydev->interrupts);

	interrupts = phydev->interrupts;

	if (interrupts == PHY_INTERRUPT_ENABLED) {
		/* enable all interrupts */
		err = phy_write(phydev, MII_INTMASK, INTERRUPT_ALL);
	} else if (interrupts == PHY_INTERRUPT_DISABLED) {
		/* disable all interrupts */
		err = phy_write(phydev, MII_INTMASK, INTERRUPT_NONE);
	} else {
		/* interpret value of interrupts as interrupt mask */
		err = phy_write(phydev, MII_INTMASK, interrupts);
	}

	if (err < 0)
		goto phy_write_error;

	/* Fix for situation in which multiple phys are connected after each
	 * other on a platform that does not support level triggered interrupts
	 * (i.e. only edge triggering):
	 * If interrupts occur during connection of the phy to a netdev,
	 * there is a race condition that can lead to interrupts not being
	 * cleared. The pending interrupt will then hog (pull to 0) a shared IRQ
	 * line (since there is no level triggering),
	 * so that the ISR is never run and interrupts are disfunctional.
	 *
	 * config_intr is the last function called by phy_connect_direct and
	 * by handling any pending interrupts here, we can avoid this.
	 */
	handle_interrupts(phydev);

	return 0;

phy_write_error:
	dev_err(&phydev->PHYDEV_DEV, "write error: %s failed\n", __func__);
	return err;
}

/* interrupt handler for pwon interrupts */
static inline void handle_pwon_interrupt(struct phy_device *phydev)
{
	if (verbosity > 0)
		dev_alert(&phydev->PHYDEV_DEV,
		"re-initializing phy [%08x] @ [%04x] after powerdown\n",
		phydev->phy_id, phydev->PHYDEV_ADDR);
	/* after a power down reinitialize the phy */
	phydev->drv->config_init(phydev);

	/* update master/slave setting */
	PHY_PRIV(phydev)->is_master = get_master_cfg(phydev);

	/* For TJA1102, pwon interrupts only exist on TJA1102p0
	 * Find TJA1102p1 to reinitialize it too
	 */
	if ((phydev->phy_id & NXP_PHY_ID_MASK) == NXP_PHY_ID_TJA1102P0) {
		int p1_addr = phydev->PHYDEV_ADDR + 1;
		struct phy_device *phydevp1;

		if (p1_addr >= PHY_MAX_ADDR)
			return;

		phydevp1 = GET_PHYDEV(phydev->PHYDEV_BUS, p1_addr);
		if (!phydevp1)
			return;

		if (verbosity > 0)
			dev_alert(&phydev->PHYDEV_DEV,
			"reinit phy [%08x] @ [%04x] after pDown\n",
			phydevp1->phy_id, phydevp1->PHYDEV_ADDR);
		phydevp1->drv->config_init(phydevp1);
		PHY_PRIV(phydevp1)->is_master = get_master_cfg(phydevp1);
	}
}

/* interrupt handler for undervoltage recovery interrupts */
static inline void handle_uvr_interrupt(struct phy_device *phydev)
{
	if (verbosity > 0)
		dev_alert(&phydev->PHYDEV_DEV,
		"resuming phy [%08x] @ [%04x] after uvr\n",
		phydev->phy_id, phydev->PHYDEV_ADDR);
	phydev->drv->resume(phydev);

	/* For TJA1102, UVR interrupts only exist on TJA1102p0
	 * Find TJA1102p1 to resume it too
	 */
	if ((phydev->phy_id & NXP_PHY_ID_MASK) == NXP_PHY_ID_TJA1102P0) {
		int p1_addr = phydev->PHYDEV_ADDR + 1;
		struct phy_device *phydevp1;

		if (p1_addr >= PHY_MAX_ADDR)
			return;

		phydevp1 = GET_PHYDEV(phydev->PHYDEV_BUS, p1_addr);
		if (!phydevp1)
			return;

		if (verbosity > 0)
			dev_alert(&phydev->PHYDEV_DEV,
			"resuming phy [%08x] @ [%04x] after uvr\n",
			phydevp1->phy_id, phydevp1->PHYDEV_ADDR);
		phydevp1->drv->resume(phydevp1);
	}
}

static int handle_interrupts(struct phy_device *phydev)
{
	int interrupts;

	/* query phy for interrupts */
	interrupts = phy_read(phydev, MII_INTSRC);
	if (interrupts < 0)
		goto phy_read_error;

	if (verbosity > 4)
		dev_alert(&phydev->PHYDEV_DEV,
		"interrupt on phy [%08x]@[%04x], ISR [%08x]\n",
		phydev->phy_id, phydev->PHYDEV_ADDR, interrupts);

	/* Handle some nxp specific interrupts here:
	 * - reinitialize after power down
	 * - resume PHY after an external WAKEUP was received
	 * - resume PHY after an undervoltage recovery
	 * - adjust state on link changes (in case we are polling)
	 * - check for some PHY errors
	 */

	if (interrupts & INTERRUPT_PWON)
		handle_pwon_interrupt(phydev);
	else if (interrupts & INTERRUPT_UV_RECOVERY)
		handle_uvr_interrupt(phydev);
	else if (interrupts & INTERRUPT_WAKEUP)
		phydev->drv->resume(phydev);

	/* warnings */
	if (interrupts & INTERRUPT_PHY_INIT_FAIL)
		dev_err(&phydev->PHYDEV_DEV, "PHY initialization failed\n");
	if (interrupts & INTERRUPT_LINK_STATUS_FAIL)
		dev_err(&phydev->PHYDEV_DEV, "PHY link status failed\n");
	if (interrupts & INTERRUPT_SYM_ERR)
		dev_err(&phydev->PHYDEV_DEV, "PHY symbol error detected\n");
	if (interrupts & INTERRUPT_SNR_WARNING)
		dev_err(&phydev->PHYDEV_DEV, "PHY SNR warning\n");
	if (interrupts & INTERRUPT_CONTROL_ERROR)
		dev_err(&phydev->PHYDEV_DEV, "PHY control error\n");
	if (interrupts & INTERRUPT_UV_ERR)
		dev_err(&phydev->PHYDEV_DEV, "PHY undervoltage error\n");
	if (interrupts & INTERRUPT_TEMP_ERROR)
		dev_err(&phydev->PHYDEV_DEV, "PHY temperature error\n");

	return interrupts;

/* error handling */
phy_read_error:
	dev_err(&phydev->PHYDEV_DEV, "read error: %s failed\n", __func__);
	return interrupts;
}

#ifdef CONFIG_POLL

/* polling function, that is executed regularly to handle phy interrupts */
static void poll(struct work_struct *work)
{
	int interrupts;
	struct phy_device *phydev =
		container_of(work, struct phy_device, phy_queue);


	interrupts = handle_interrupts(phydev);

	/* Notify state machine about any link changes */
	if (interrupts & INTERRUPT_LINK_STATUS_UP ||
	    interrupts & INTERRUPT_LINK_STATUS_FAIL) {

		/* This would be done by phy_change() if interrupts were used.
		 * Only indicate a link change to state machine
		 * if phydev is attached to a netdevice
		 */
		if (phydev->attached_dev && (PHY_RUNNING == phydev->state ||
		    PHY_NOLINK == phydev->state)) {

			mutex_lock(&phydev->lock);
			phydev->state = PHY_CHANGELINK;
			mutex_unlock(&phydev->lock);
		}

		if (verbosity > 1)
			dev_alert(&phydev->PHYDEV_DEV,
			"state was %d, now going %s\n", phydev->state,
			(interrupts & INTERRUPT_LINK_STATUS_UP) ?
			"UP":"DOWN");

/* if we are in standalone mode, the state machine is not
 * running, so there is no need to reschedule it
 */
#ifndef CONFIG_STANDALONE_PHY
		/* reschedule state queue work to run as soon as possible */
		cancel_delayed_work_sync(&phydev->state_queue);
		queue_delayed_work(system_power_efficient_wq, &phydev->state_queue, 0);
#endif
	}

	/* requeue poll function */
	msleep(POLL_PAUSE);	/* msleep is non-blocking */
	queue_work(system_power_efficient_wq, &phydev->phy_queue);
}

static void setup_polling(struct phy_device *phydev)
{
	/*
	 * The phy_queue is normally used to schedule the interrupt handler
	 * from interrupt context after an irq has been received.
	 * Here it is repurposed as scheduling mechanism for the poll function
	 */
	if (!PHY_PRIV(phydev)->is_poll_setup) {
		if (verbosity > 0)
			dev_alert(&phydev->PHYDEV_DEV,
			"initialize polling for PHY %x\n", phydev->PHYDEV_ADDR);
		cancel_work_sync(&phydev->phy_queue);
		INIT_WORK(&phydev->phy_queue, poll);
		PHY_PRIV(phydev)->is_poll_setup = true;
	}
}

static void start_polling(struct phy_device *phydev)
{
	if (PHY_PRIV(phydev)->is_poll_setup && !PHY_PRIV(phydev)->is_polling) {
		if (verbosity > 0)
			dev_alert(&phydev->PHYDEV_DEV, "start polling PHY %x\n",
			phydev->PHYDEV_ADDR);
		/* schedule execution of polling function */
		queue_work(system_power_efficient_wq, &phydev->phy_queue);
		PHY_PRIV(phydev)->is_polling = true;
	}
}

static void stop_polling(struct phy_device *phydev)
{
	if (PHY_PRIV(phydev)->is_poll_setup && PHY_PRIV(phydev)->is_polling) {
		if (verbosity > 0)
			dev_alert(&phydev->PHYDEV_DEV, "stop polling PHY %x\n",
			phydev->PHYDEV_ADDR);
		/* cancel scheduled work */
		cancel_work_sync(&phydev->phy_queue);
		PHY_PRIV(phydev)->is_polling = false;
	}
}

#endif

/* helper function, waits until a given condition is met
 *
 * The function delays until the part of the register at reg_addr,
 * defined by reg_mask equals cond, or a timeout (timeout*DELAY_LENGTH) occurs.
 * @return	0 if condition is met, <0 if timeout or read error occurred
 */
static int wait_on_condition(struct phy_device *phydev, int reg_addr,
			     int reg_mask, int cond, int timeout)
{
	int reg_val;

	if (verbosity > 3)
		dev_alert(&phydev->PHYDEV_DEV, "waiting on condition\n");

	do {
		udelay(DELAY_LENGTH);
		reg_val = phy_read(phydev, reg_addr);
		if (reg_val < 0)
			return reg_val;
	} while ((reg_val & reg_mask) != cond && --timeout);

	if (verbosity > 3)
		dev_alert(&phydev->PHYDEV_DEV, "%s",
		(timeout?"condition met\n" : "timeout occurred\n"));

	if (timeout)
		return 0;
	return -1;
}

/* helper function, enables or disables link control */
static void set_link_control(struct phy_device *phydev, int enable_link_control)
{
	int err;

	err = phy_configure_bit(phydev, MII_ECTRL, ECTRL_LINK_CONTROL,
				enable_link_control);
	if (err < 0)
		goto phy_configure_error;
	if (verbosity > 1)
		dev_alert(&phydev->PHYDEV_DEV,
		"set link ctrl to [%d] for phy %x completed\n",
		enable_link_control, phydev->PHYDEV_ADDR);

	return;

phy_configure_error:
	dev_err(&phydev->PHYDEV_DEV, "phy r/w error: setting link control failed\n");
}

/* Helper function, configures phy as master or slave
 * @param  phydev    the phy to be configured
 * @param  setMaster ==0: set to slave
 *                   !=0: set to master
 * @return           0 on success, error code on failure
 */
static int set_master_cfg(struct phy_device *phydev, int setMaster)
{
	int err;

	/* disable link control prior to master/slave cfg */
	set_link_control(phydev, 0);

	/* write configuration to the phy */
	err = phy_configure_bit(phydev, MII_CFG1, CFG1_MASTER_SLAVE, setMaster);
	if (err < 0)
		goto phy_configure_error;

	if (verbosity > 1)
		dev_alert(&phydev->PHYDEV_DEV, "set master cfg completed\n");

	/* enable link control after master/slave cfg was set */
	set_link_control(phydev, 1);

	return 0;

/* error handling */
phy_configure_error:
	dev_err(&phydev->PHYDEV_DEV, "phy r/w error: %s failed\n", __func__);
	return err;
}

/* Helper function, reads master/slave configuration of phy
 * @param  phydev    the phy to be read
 *
 * @return           ==0: is slave
 *                   !=0: is master
 */
static bool get_master_cfg(struct phy_device *phydev)
{
	int reg_val;

	if (verbosity > 1)
		dev_alert(&phydev->PHYDEV_DEV, "getting master cfg PHY %x\n",
		phydev->PHYDEV_ADDR);

	/* read the current configuration */
	reg_val = phy_read(phydev, MII_CFG1);
	if (reg_val < 0)
		goto phy_read_error;

	return reg_val & CFG1_MASTER_SLAVE;

/* error handling */
phy_read_error:
	dev_err(&phydev->PHYDEV_DEV, "read error: %s failed\n", __func__);
	return reg_val;
}

/* retrieves the link status from COMMSTAT register */
static int get_link_status(struct phy_device *phydev)
{
	int reg_val;

	reg_val = phy_read(phydev, MII_COMMSTAT);
	if (reg_val < 0)
		goto phy_read_error;

	return reg_val & COMMSTAT_LINK_UP;

/* error handling */
phy_read_error:
	dev_err(&phydev->PHYDEV_DEV, "read error: %s failed\n", __func__);
	return reg_val;
}

/* issues a sleep request, if in managed mode */
static int nxp_sleep(struct phy_device *phydev)
{
	int err;

	if (verbosity > 0)
		dev_alert(&phydev->PHYDEV_DEV, "PHY %x going to sleep\n",
		phydev->PHYDEV_ADDR);

	if (!managed_mode)
		goto phy_auto_op_error;

	/* clear power mode bits and set them to sleep request */
	err = phy_configure_bits(phydev, MII_ECTRL, ECTRL_POWER_MODE,
				 POWER_MODE_SLEEPREQUEST);
	if (err < 0)
		goto phy_configure_error;

	if ((phydev->phy_id & NXP_PHY_ID_MASK) == NXP_PHY_ID_TJA1102P0 ||
	    (phydev->phy_id & NXP_PHY_ID_MASK) == NXP_PHY_ID_TJA1102P1 ||
	    (phydev->phy_id & NXP_PHY_ID_MASK) == NXP_PHY_ID_TJA1101) {
		/* tja1102 and tja1102 have an extra sleep state indicator
		 * in ECTRL.
		 * If transition is successful this can be detected immediately,
		 * without waiting for SLEEP_REQ_TIMEOUT to pass
		 */
		err = wait_on_condition(phydev, MII_ECTRL, ECTRL_POWER_MODE,
					POWER_MODE_SLEEP, SLEEP_REQ_TIMEOUT);
		if (err < 0)
			goto phy_transition_error;
	} else if ((phydev->phy_id & NXP_PHY_ID_MASK) == NXP_PHY_ID_TJA1100) {
		/* TJA1100 disables SMI when entering SLEEP
		 * The SMI bus is pulled up, that means every
		 * SMI read will return 0xffff.
		 * We can use this to check if PHY entered SLEEP.
		 */
		err = wait_on_condition(phydev, MII_ECTRL,
					0xffff, 0xffff, SLEEP_REQ_TIMEOUT);
		if (err < 0)
			goto phy_transition_error;
	}

	return 0;

/* error handling */
phy_auto_op_error:
	dev_info(&phydev->PHYDEV_DEV, "phy is in auto mode: sleep not possible\n");
	return 0;

phy_configure_error:
	dev_err(&phydev->PHYDEV_DEV, "phy r/w error: entering sleep failed\n");
	return err;

phy_transition_error:
	dev_err(&phydev->PHYDEV_DEV, "sleep request timed out\n");
	return err;
}

/* wakes up the phy from sleep mode */
static int wakeup_from_sleep(struct phy_device *phydev)
{
	int err;
	unsigned long wakeup_delay;

	if (verbosity > 0)
		dev_alert(&phydev->PHYDEV_DEV, "PHY %x waking up from sleep\n",
		phydev->PHYDEV_ADDR);

	if (!managed_mode)
		goto phy_auto_op_error;

	/* set power mode bits to standby mode */
	err = phy_configure_bits(phydev, MII_ECTRL, ECTRL_POWER_MODE,
				 POWER_MODE_STANDBY);
	if (err < 0)
		goto phy_configure_error;

	/* wait until power mode transition is completed */
	err = wait_on_condition(phydev, MII_ECTRL, ECTRL_POWER_MODE,
				POWER_MODE_STANDBY, POWER_MODE_TIMEOUT);
	if (err < 0)
		goto phy_transition_error;

	/* set power mode bits to normal mode */
	err = phy_configure_bits(phydev, MII_ECTRL, ECTRL_POWER_MODE,
				 POWER_MODE_NORMAL);
	if (err < 0)
		goto phy_configure_error;

	/* wait until the PLL is locked, indicating a completed transition */
	err = wait_on_condition(phydev, MII_GENSTAT, GENSTAT_PLL_LOCKED,
				GENSTAT_PLL_LOCKED, POWER_MODE_TIMEOUT);
	if (err < 0)
		goto phy_transition_error;
	/* if phy is configured as slave, also send a wakeup request
	 * to master
	 */
	if (!PHY_PRIV(phydev)->is_master) {
		if (verbosity > 0)
			dev_alert(&phydev->PHYDEV_DEV,
			"Phy is slave, send wakeup request master\n");
		/* link control must be reset for wake request */
		set_link_control(phydev, 0);

		/* start sending bus wakeup signal */
		err = phy_configure_bit(phydev, MII_ECTRL,
					ECTRL_WAKE_REQUEST, 1);
		if (err < 0)
			goto phy_configure_error;

		switch (phydev->phy_id & NXP_PHY_ID_MASK) {
		case NXP_PHY_ID_TJA1100:
			wakeup_delay = TJA100_WAKE_REQ_TIMEOUT;
			break;
		case NXP_PHY_ID_TJA1102P0:
			/* fall through */
		case NXP_PHY_ID_TJA1101:
			/* fall through */
		case NXP_PHY_ID_TJA1102P1:
			wakeup_delay = TJA102_WAKE_REQ_TIMEOUT;
			break;
		default:
			goto unsupported_phy_error;
		}

		/* wait until link partner is guaranteed to be awake */
		usleep_range(wakeup_delay, wakeup_delay + 1U);

		/* stop sending bus wakeup signal */
		err = phy_configure_bit(phydev, MII_ECTRL,
					ECTRL_WAKE_REQUEST, 0);
		if (err < 0)
			goto phy_configure_error;
	}

	/* reenable link control */
	set_link_control(phydev, 1);

	return 0;

/* error handling */
phy_auto_op_error:
	dev_dbg(&phydev->PHYDEV_DEV, "phy is in auto mode: wakeup not possible\n");
	return 0;

phy_configure_error:
	dev_err(&phydev->PHYDEV_DEV, "phy r/w error: wakeup failed\n");
	return err;

phy_transition_error:
	dev_err(&phydev->PHYDEV_DEV, "power mode transition failed\n");
	return err;

unsupported_phy_error:
	dev_err(&phydev->PHYDEV_DEV, "unsupported phy, wakeup failed\n");
	return -1;
}

/* send a wakeup request to the link partner */
static int wakeup_from_normal(struct phy_device *phydev)
{
	int err;

	if (verbosity > 0)
		dev_alert(&phydev->PHYDEV_DEV,
		"PHY %x waking up from normal (send wur)\n", phydev->PHYDEV_ADDR);

	/* start sending bus wakeup signal */
	err = phy_configure_bit(phydev, MII_ECTRL, ECTRL_WAKE_REQUEST, 1);
	if (err < 0)
		goto phy_configure_error;

	/* stop sending bus wakeup signal */
	err = phy_configure_bit(phydev, MII_ECTRL, ECTRL_WAKE_REQUEST, 0);
	if (err < 0)
		goto phy_configure_error;

	return 0;

/* error handling */
phy_configure_error:
	dev_err(&phydev->PHYDEV_DEV, "phy r/w error: %s failed\n", __func__);
	return err;
}

/* wake up phy if is in sleep mode, send wakeup request if in normal mode */
static int nxp_wakeup(struct phy_device *phydev)
{
	int reg_val;
	int err = 0;

	reg_val = phy_read(phydev, MII_ECTRL);
	if (reg_val < 0)
		goto phy_read_error;

	reg_val &= ECTRL_POWER_MODE;
	switch (reg_val) {
	case POWER_MODE_NORMAL:
		err = wakeup_from_normal(phydev);
		break;
	case POWER_MODE_SLEEP:
		err = wakeup_from_sleep(phydev);
		break;
	case 0xffff & ECTRL_POWER_MODE:
		/* TJA1100 disables SMI during sleep */
		goto phy_SMI_disabled;
	default:
		break;
	}
	if (err < 0)
		goto phy_configure_error;

	return 0;

/* error handling */
phy_read_error:
	dev_err(&phydev->PHYDEV_DEV, "read error: %s failed\n", __func__);
	return reg_val;

phy_SMI_disabled:
	dev_err(&phydev->PHYDEV_DEV, "SMI interface disabled, cannot be woken up\n");
	return 0;

phy_configure_error:
	dev_err(&phydev->PHYDEV_DEV, "phy r/w error: %s failed\n", __func__);
	return err;
}

/* power mode transition to standby */
static int nxp_suspend(struct phy_device *phydev)
{
	int err;

	if (verbosity > 0)
		dev_alert(&phydev->PHYDEV_DEV, "suspending PHY %x\n", phydev->PHYDEV_ADDR);

	if (!managed_mode)
		goto phy_auto_op_error;

	WARN_ON(mutex_is_locked(&phydev->lock));
	mutex_lock(&phydev->lock);
	/* set BMCR_PDOWN bit in MII_BMCR register */
	err = phy_configure_bit(phydev, MII_BMCR, BMCR_PDOWN, 1);
	if (err < 0)
		goto phy_configure_error;
	mutex_unlock(&phydev->lock);

	return 0;

/* error handling */
phy_configure_error:
	mutex_unlock(&phydev->lock);
	dev_err(&phydev->PHYDEV_DEV, "phy r/w error: %s failed\n", __func__);
	return err;

phy_auto_op_error:
	dev_dbg(&phydev->PHYDEV_DEV, "phy is in auto mode: suspend not possible\n");
	return 0;
}

/* power mode transition from standby to normal */
static int nxp_resume(struct phy_device *phydev)
{
	int err;

	if (verbosity > 0)
		dev_alert(&phydev->PHYDEV_DEV, "resuming PHY %x\n", phydev->PHYDEV_ADDR);

	/* Since v4.14.28 .resume() is supposed to be called with mutex
	 * phydev->lock already locked.
	 * In previous versions, it must be locked here
	 */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,14,27)
	WARN_ON(mutex_is_locked(&phydev->lock));
	mutex_lock(&phydev->lock);
#endif
	/* clear BMCR_PDOWN bit in MII_BMCR register */
	err = phy_configure_bit(phydev, MII_BMCR, BMCR_PDOWN, 0);
	if (err < 0)
		goto phy_configure_error;

	/* transit to normal mode */
	err = phy_configure_bits(phydev, MII_ECTRL, ECTRL_POWER_MODE,
				 POWER_MODE_NORMAL);
	if (err < 0)
		goto phy_configure_error;

	/* wait until power mode transition is completed */
	err = wait_on_condition(phydev, MII_ECTRL, ECTRL_POWER_MODE,
				POWER_MODE_NORMAL, POWER_MODE_TIMEOUT);
	if (err < 0)
		goto phy_transition_error;

	/* wait until the PLL is locked, indicating a completed transition */
	err = wait_on_condition(phydev, MII_GENSTAT, GENSTAT_PLL_LOCKED,
				GENSTAT_PLL_LOCKED, POWER_MODE_TIMEOUT);
	if (err < 0)
		goto phy_pll_error;

	/* reenable link control */
	set_link_control(phydev, 1);
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,14,27)
	mutex_unlock(&phydev->lock);
#endif

	return 0;

/* error handling */
phy_configure_error:
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,14,27)
	mutex_unlock(&phydev->lock);
#endif
	dev_err(&phydev->PHYDEV_DEV, "phy r/w error: %s failed\n", __func__);
	return err;

phy_transition_error:
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,14,27)
	mutex_unlock(&phydev->lock);
#endif
	dev_err(&phydev->PHYDEV_DEV, "power mode transition failed\n");
	return err;

phy_pll_error:
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,14,27)
	mutex_unlock(&phydev->lock);
#endif
	dev_err(&phydev->PHYDEV_DEV, "Error: PLL is unstable and not locked\n");
	return err;
}

/* Configures the autonegotiation capabilities */
static int nxp_config_aneg(struct phy_device *phydev)
{
	if (verbosity > 0)
		dev_alert(&phydev->PHYDEV_DEV, "configuring autoneg\n");

	/* disable autoneg and manually configure speed, duplex, pause frames */
	phydev->autoneg = 0;

	phydev->speed = SPEED_100;
	phydev->duplex = DUPLEX_FULL;

	phydev->pause = 0;
	phydev->asym_pause = 0;

	return 0;
}

/* helper function, enters the test mode specified by tmode
 * @return          0 if test mode was entered, <0 on read or write error
 */
static int enter_test_mode(struct phy_device *phydev, enum test_mode tmode)
{
	int reg_val = -1;
	int err;

	if (verbosity > 1)
		dev_alert(&phydev->PHYDEV_DEV, "phy %x entering test mode: %d\n",
		phydev->PHYDEV_ADDR, tmode);
	switch (tmode) {
	case NO_TMODE:
		reg_val = ECTRL_NO_TMODE;
		break;
	case TMODE1:
		reg_val = ECTRL_TMODE1;
		break;
	case TMODE2:
		reg_val = ECTRL_TMODE2;
		break;
	case TMODE3:
		reg_val = ECTRL_TMODE3;
		break;
	case TMODE4:
		reg_val = ECTRL_TMODE4;
		break;
	case TMODE5:
		reg_val = ECTRL_TMODE5;
		break;
	case TMODE6:
		reg_val = ECTRL_TMODE6;
		break;
	default:
		break;
	}

	if (reg_val >= 0) {
		/* set test mode bits accordingly */
		err = phy_configure_bits(phydev, MII_ECTRL, ECTRL_TEST_MODE,
					 reg_val);
		if (err < 0)
			goto phy_configure_error;
	}

	return 0;

/* error handling */
phy_configure_error:
	dev_err(&phydev->PHYDEV_DEV, "phy r/w error: setting test mode failed\n");
	return err;
}

/* helper function, enables or disables loopback mode
 * @return	0 if loopback mode was configured, <0 on read or write error
 */
static int set_loopback(struct phy_device *phydev, int enable_loopback)
{
	int err;

	if (verbosity > 1)
		dev_alert(&phydev->PHYDEV_DEV, "phy %x setting loopback: %d\n",
		phydev->PHYDEV_ADDR, enable_loopback);
	err = phy_configure_bit(phydev, MII_BMCR, BMCR_LOOPBACK,
				enable_loopback);
	if (err < 0)
		goto phy_configure_error;

	return 0;

/* error handling */
phy_configure_error:
	dev_err(&phydev->PHYDEV_DEV, "phy r/w error: configuring loopback failed\n");
	return err;
}

/* helper function, enters the loopback mode specified by lmode
 * @return          0 if loopback mode was entered, <0 on read or write error
 */
static int enter_loopback_mode(struct phy_device *phydev,
			       enum loopback_mode lmode)
{
	int reg_val = -1;
	int err;

	/* disable link control prior to loopback cfg */
	set_link_control(phydev, 0);

	switch (lmode) {
	case NO_LMODE:
		if (verbosity > 1)
			dev_alert(&phydev->PHYDEV_DEV,
			"phy %x disabling loopback mode\n", phydev->PHYDEV_ADDR);
		/* disable loopback */
		err = set_loopback(phydev, 0);
		if (err < 0)
			goto phy_set_loopback_error;
		break;
	case INTERNAL_LMODE:
		reg_val = ECTRL_INTERNAL_LMODE;
		break;
	case EXTERNAL_LMODE:
		reg_val = ECTRL_EXTERNAL_LMODE;
		break;
	case REMOTE_LMODE:
		reg_val = ECTRL_REMOTE_LMODE;
		break;
	default:
		break;
	}

	if (reg_val >= 0) {
		if (verbosity > 1)
			dev_alert(&phydev->PHYDEV_DEV, "setting loopback mode %d\n",
			lmode);
		err = phy_configure_bits(phydev, MII_ECTRL,
					 ECTRL_LOOPBACK_MODE, reg_val);
		if (err < 0)
			goto phy_configure_error;

		/* enable loopback */
		err = set_loopback(phydev, 1);
		if (err < 0)
			goto phy_set_loopback_error;
	}

	/* enable link control after loopback cfg was set */
	set_link_control(phydev, 1);

	return 0;

/* error handling */
phy_set_loopback_error:
	dev_err(&phydev->PHYDEV_DEV, "error: enable/disable loopback failed\n");
	return err;

phy_configure_error:
	dev_err(&phydev->PHYDEV_DEV, "phy r/w error: setting loopback mode failed\n");
	return err;
}

/* enable or disable loopback mode */
static int __maybe_unused nxp_set_loopback(struct phy_device *phydev, bool enable)
{
	int ret;

	if (enable)
		ret = enter_loopback_mode(phydev, INTERNAL_LMODE);
	else
		ret = enter_loopback_mode(phydev, NO_LMODE);

	return ret;
}

/* helper function, enters the led mode specified by lmode
 * @return          0 if led mode was entered, <0 on read or write error
 */
static int enter_led_mode(struct phy_device *phydev, enum led_mode lmode)
{
	int reg_val = -1;
	int err;

	switch (lmode) {
	case NO_LED_MODE:
		/* disable led */
		err = phy_configure_bit(phydev, MII_CFG1,
					TJA1100_CFG1_LED_EN, 0);
		if (err < 0)
			goto phy_configure_error;
		break;
	case LINKUP_LED_MODE:
		reg_val = CFG1_LED_LINKUP;
		break;
	case FRAMEREC_LED_MODE:
		reg_val = CFG1_LED_FRAMEREC;
		break;
	case SYMERR_LED_MODE:
		reg_val = CFG1_LED_SYMERR;
		break;
	case CRSSIG_LED_MODE:
		reg_val = CFG1_LED_CRSSIG;
		break;
	default:
		break;
	}

	if (reg_val >= 0) {
		err = phy_configure_bits(phydev, MII_CFG1,
					 TJA1100_CFG1_LED_MODE, reg_val);
		if (err < 0)
			goto phy_configure_error;

		/* enable led */
		err = phy_configure_bit(phydev, MII_CFG1,
					TJA1100_CFG1_LED_EN, 1);
		if (err < 0)
			goto phy_configure_error;
	}

	return 0;

/* error handling */
phy_configure_error:
	dev_err(&phydev->PHYDEV_DEV, "phy r/w error: setting led mode failed\n");
	return err;
}

/* This function handles read accesses to the node 'master_cfg' in
 * sysfs.
 * Depending on current configuration of the phy, the node reads
 * 'master' or 'slave'
 */
static ssize_t sysfs_get_master_cfg(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	bool is_master;
	struct phy_device *phydev = container_of(dev, struct phy_device, PHYDEV_DEV);

	is_master = get_master_cfg(phydev);

	/* write result into the buffer */
	return scnprintf(buf, PAGE_SIZE, "%s\n",
			 is_master ? "master" : "slave");
}

/* This function handles write accesses to the node 'master_cfg' in sysfs.
 * Depending on the value written to it, the phy is configured as
 * master or slave
 */
static ssize_t sysfs_set_master_cfg(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int err;
	int setMaster;
	struct phy_device *phydev = container_of(dev, struct phy_device, PHYDEV_DEV);

	if (verbosity > 1)
		dev_alert(&phydev->PHYDEV_DEV, "setting master cfg PHY %x\n",
		phydev->PHYDEV_ADDR);

	/* parse the buffer */
	err = kstrtoint(buf, 10, &setMaster);
	if (err < 0)
		goto phy_parse_error;

	/* write configuration to the phy */
	err = set_master_cfg(phydev, setMaster);
	if (err < 0)
		goto phy_cfg_error;

	/* update phydev */
	PHY_PRIV(phydev)->is_master = !!setMaster;

	return count;

/* error handling */
phy_parse_error:
	dev_err(&phydev->PHYDEV_DEV, "parse error: %s failed\n", __func__);
	return err;

phy_cfg_error:
	dev_err(&phydev->PHYDEV_DEV, "phy cfg error: %s failed\n", __func__);
	return err;
}

/* This function handles read accesses to the node 'power_cfg' in sysfs.
 * Reading the node returns the current power state
 */
static ssize_t sysfs_get_power_cfg(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int reg_val;
	char *pmode;
	struct phy_device *phydev = container_of(dev, struct phy_device, PHYDEV_DEV);

	if (verbosity > 1)
		dev_alert(&phydev->PHYDEV_DEV, "getting power cfg\n");

	reg_val = phy_read(phydev, MII_ECTRL);
	if (reg_val < 0)
		goto phy_read_error;

	/* mask power mode bits */
	reg_val &= ECTRL_POWER_MODE;

	switch (reg_val) {
	case POWER_MODE_NORMAL:
		pmode = "POWER_MODE_NORMAL\n";
		break;
	case POWER_MODE_SLEEPREQUEST:
		pmode = "POWER_MODE_SLEEPREQUEST\n";
		break;
	case POWER_MODE_SLEEP:
		pmode = "POWER_MODE_SLEEP\n";
		break;
	case POWER_MODE_SILENT:
		pmode = "POWER_MODE_SILENT\n";
		break;
	case POWER_MODE_STANDBY:
		pmode = "POWER_MODE_STANDBY\n";
		break;
	case POWER_MODE_NOCHANGE:
		pmode = "POWER_MODE_NOCHANGE\n";
		break;
	default:
		if (verbosity > 1)
			dev_alert(&phydev->PHYDEV_DEV,
			"unknown reg val is [%08x]\n", reg_val);
		pmode = "unknown\n";
	}

	/* write result into the buffer */
	return scnprintf(buf, PAGE_SIZE, pmode);

/* error handling */
phy_read_error:
	dev_err(&phydev->PHYDEV_DEV, "read error: %s failed\n", __func__);
	return reg_val;
}

/* This function handles write accesses to the node 'power_cfg' in
 * sysfs.
 * Depending on the value written to it, the phy enters a certain
 * power state.
 */
static ssize_t sysfs_set_power_cfg(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int err;
	int pmode;
	struct phy_device *phydev = container_of(dev, struct phy_device, PHYDEV_DEV);

	/* parse the buffer */
	err = kstrtoint(buf, 10, &pmode);
	if (err < 0)
		goto phy_parse_error;

	if (verbosity > 1)
		dev_alert(&phydev->PHYDEV_DEV, "set pmode to %d\n", pmode);

	switch (pmode) {
	case 0:
		err = phydev->drv->suspend(phydev);
		break;
	case 1:
		err = phydev->drv->resume(phydev);
		break;
	case 2:
		err = nxp_sleep(phydev);
		break;
	case 3:
		err = nxp_wakeup(phydev);
		break;
	default:
		break;
	}

	if (err)
		goto phy_pmode_transit_error;

	return count;

/* error handling */
phy_parse_error:
	dev_err(&phydev->PHYDEV_DEV, "parse error: %s failed\n", __func__);
	return err;

phy_pmode_transit_error:
	dev_err(&phydev->PHYDEV_DEV, "pmode error: %s failed\n", __func__);
	return err;
}

/* This function handles read accesses to the node 'loopback_cfg' in sysfs
 * Reading the node returns the current loopback configuration
 */
static ssize_t sysfs_get_loopback_cfg(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	int reg_val;
	char *lmode;
	struct phy_device *phydev = container_of(dev, struct phy_device, PHYDEV_DEV);

	if (verbosity > 1)
		dev_alert(&phydev->PHYDEV_DEV, "getting loopback cfg\n");

	reg_val = phy_read(phydev, MII_BMCR);
	if (reg_val < 0)
		goto phy_read_error;

	if (reg_val & BMCR_LOOPBACK) {
		/* loopback enabled */
		reg_val = phy_read(phydev, MII_ECTRL);
		if (reg_val < 0)
			goto phy_read_error;

		/* mask loopback mode bits */
		reg_val &= ECTRL_LOOPBACK_MODE;

		switch (reg_val) {
		case ECTRL_INTERNAL_LMODE:
			lmode = "INTERNAL_LOOPBACK\n";
			break;
		case ECTRL_EXTERNAL_LMODE:
			lmode = "EXTERNAL_LOOPBACK\n";
			break;
		case ECTRL_REMOTE_LMODE:
			lmode = "REMOTE_LOOPBACK\n";
			break;
		default:
			lmode = "unknown\n";
			if (verbosity > 1)
				dev_alert(&phydev->PHYDEV_DEV,
				"unknown reg val is [%08x]\n", reg_val);
		}
	} else {
		/* loopback disabled */
		lmode = "LOOPBACK_DISABLED\n";
	}

	/* write result into the buffer */
	return scnprintf(buf, PAGE_SIZE, lmode);

/* error handling */
phy_read_error:
	dev_err(&phydev->PHYDEV_DEV, "read error: %s failed\n", __func__);
	return reg_val;
}

/* This function handles write accesses to the node 'loopback_cfg'
 * in sysfs.
 * Depending on the value written to it, the phy enters a certain
 * loopback state.
 */
static ssize_t sysfs_set_loopback_cfg(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	int err;
	int lmode;
	struct phy_device *phydev = container_of(dev, struct phy_device, PHYDEV_DEV);

	if (!managed_mode)
		goto phy_auto_op_error;

	if (verbosity > 1)
		dev_alert(&phydev->PHYDEV_DEV, "setting loopback cfg PHY %x\n",
		phydev->PHYDEV_ADDR);

	/* parse the buffer */
	err = kstrtoint(buf, 10, &lmode);
	if (err < 0)
		goto phy_parse_error;

	switch (lmode) {
	case 0:
		err = enter_loopback_mode(phydev, NO_LMODE);
#ifdef CONFIG_POLL
		if (!no_poll)
			start_polling(phydev);
#endif
		break;
	case 1:
#ifdef CONFIG_POLL
		if (!no_poll)
			stop_polling(phydev);
#endif
		err = enter_loopback_mode(phydev, INTERNAL_LMODE);
		break;
	case 2:
#ifdef CONFIG_POLL
		if (!no_poll)
			stop_polling(phydev);
#endif
		err = enter_loopback_mode(phydev, EXTERNAL_LMODE);
		break;
	case 3:
#ifdef CONFIG_POLL
		if (!no_poll)
			stop_polling(phydev);
#endif
		err = enter_loopback_mode(phydev, REMOTE_LMODE);
		break;
	default:
		break;
	}

	if (err)
		goto phy_lmode_transit_error;

	return count;

/* error handling */
phy_auto_op_error:
	dev_info(&phydev->PHYDEV_DEV, "phy is in auto mode: loopback not available\n");
	return count;

phy_parse_error:
	dev_err(&phydev->PHYDEV_DEV, "parse error: %s failed\n", __func__);
	return err;

phy_lmode_transit_error:
	dev_err(&phydev->PHYDEV_DEV, "lmode error: %s failed\n", __func__);
	return err;
}

/* This function handles read accesses to the node 'cable_test' in sysfs
 * Reading the node executes a cable test and returns the result
 */
static ssize_t sysfs_get_cable_test(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	int reg_val;
	int err;
	char *c_test_result;
	struct phy_device *phydev = container_of(dev, struct phy_device, PHYDEV_DEV);

	if (!managed_mode)
		goto phy_auto_op_error;

	if (verbosity > 1)
		dev_alert(&phydev->PHYDEV_DEV, "phy %x executing cable test\n",
		phydev->PHYDEV_ADDR);

	/* disable link control prior to cable test */
	set_link_control(phydev, 0);

	/* execute a cable test */
	err = phy_configure_bit(phydev, MII_ECTRL, ECTRL_CABLE_TEST, 1);
	if (err < 0)
		goto phy_configure_error;

	/* wait until test is completed */
	err = wait_on_condition(phydev, MII_ECTRL, ECTRL_CABLE_TEST,
				0, CABLE_TEST_TIMEOUT);
	if (err < 0)
		goto phy_transition_error;

	/* evaluate the test results */
	reg_val = phy_read(phydev, MII_EXTERNAL_STATUS);
	if (reg_val < 0)
		goto phy_read_error;

	if (reg_val & EXTSTAT_SHORT_DETECT)
		c_test_result = "SHORT_DETECT\n";
	else if (reg_val & EXTSTAT_OPEN_DETECT)
		c_test_result = "OPEN_DETECT\n";
	else
		c_test_result = "NO_ERROR\n";

	/* reenable link control after cable test */
	set_link_control(phydev, 1);

	/* write result into the buffer */
	return scnprintf(buf, PAGE_SIZE, c_test_result);

/* error handling */
phy_auto_op_error:
	dev_info(&phydev->PHYDEV_DEV, "phy is in auto mode: cabletest not available\n");
	return 0;

phy_read_error:
	dev_err(&phydev->PHYDEV_DEV, "read error: %s failed\n", __func__);
	return reg_val;

phy_configure_error:
	dev_err(&phydev->PHYDEV_DEV, "phy r/w error: %s failed\n", __func__);
	return err;

phy_transition_error:
	dev_err(&phydev->PHYDEV_DEV, "Timeout: cable test failed to finish in time\n");
	return err;
}

/* This function handles read accesses to the node 'test_mode' in sysfs
 * Reading the node returns the current test mode configuration
 */
static ssize_t sysfs_get_test_mode(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int reg_val;
	char *tmode;
	struct phy_device *phydev = container_of(dev, struct phy_device, PHYDEV_DEV);

	reg_val = phy_read(phydev, MII_ECTRL);
	if (reg_val < 0)
		goto phy_read_error;

	/* mask test mode bits */
	reg_val &= ECTRL_TEST_MODE;

	switch (reg_val) {
	case ECTRL_NO_TMODE:
		tmode = "NO_TMODE\n";
		break;
	case ECTRL_TMODE1:
		tmode = "TMODE1\n";
		break;
	case ECTRL_TMODE2:
		tmode = "TMODE2\n";
		break;
	case ECTRL_TMODE3:
		tmode = "TMODE3\n";
		break;
	case ECTRL_TMODE4:
		tmode = "TMODE4\n";
		break;
	case ECTRL_TMODE5:
		tmode = "TMODE5\n";
		break;
	case ECTRL_TMODE6:
		tmode = "TMODE6\n";
		break;
	default:
		tmode = "unknown\n";
		if (verbosity > 1)
			dev_alert(&phydev->PHYDEV_DEV,
			"unknown reg val is [%08x]\n", reg_val);
	}

	/* write result into the buffer */
	return scnprintf(buf, PAGE_SIZE, tmode);

/* error handling */
phy_read_error:
	dev_err(&phydev->PHYDEV_DEV, "read error: %s failed\n", __func__);
	return reg_val;
}

/* This function handles write accesses to the node 'test_mode' in sysfs
 * Depending on the value written to it, the phy enters a certain test mode
 */
static ssize_t sysfs_set_test_mode(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int err;
	int tmode;
	struct phy_device *phydev = container_of(dev, struct phy_device, PHYDEV_DEV);

	if (!managed_mode)
		goto phy_auto_op_error;

	/* parse the buffer */
	err = kstrtoint(buf, 10, &tmode);
	if (err < 0)
		goto phy_parse_error;

	switch (tmode) {
	case 0:
		err = enter_test_mode(phydev, NO_TMODE);
		/* enable link control after exiting test */
		set_link_control(phydev, 1);
		break;
	case 1:
		/* disbale link control before entering test */
		set_link_control(phydev, 0);
		err = enter_test_mode(phydev, TMODE1);
		break;
	case 2:
		/* disbale link control before entering test */
		set_link_control(phydev, 0);
		err = enter_test_mode(phydev, TMODE2);
		break;
	case 3:
		/* disbale link control before entering test */
		set_link_control(phydev, 0);
		err = enter_test_mode(phydev, TMODE3);
		break;
	case 4:
		/* disbale link control before entering test */
		set_link_control(phydev, 0);
		err = enter_test_mode(phydev, TMODE4);
		break;
	case 5:
		/* disbale link control before entering test */
		set_link_control(phydev, 0);
		err = enter_test_mode(phydev, TMODE5);
		break;
	case 6:
		/* disbale link control before entering test */
		set_link_control(phydev, 0);
		err = enter_test_mode(phydev, TMODE6);
		break;
	default:
		break;
	}

	if (err)
		goto phy_tmode_transit_error;

	return count;

/* error handling */
phy_auto_op_error:
	dev_info(&phydev->PHYDEV_DEV, "phy is in auto mode: testmodes not available\n");
	return count;

phy_parse_error:
	dev_err(&phydev->PHYDEV_DEV, "parse error: %s failed\n", __func__);
	return err;

phy_tmode_transit_error:
	dev_err(&phydev->PHYDEV_DEV, "tmode error: %s failed\n", __func__);
	return err;
}

/* This function handles read accesses to the node 'led_cfg' in sysfs.
 * Reading the node returns the current led configuration
 */
static ssize_t sysfs_get_led_cfg(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int reg_val;
	char *lmode;
	struct phy_device *phydev = container_of(dev, struct phy_device, PHYDEV_DEV);

	lmode = "DISABLED\n";

	/* only TJA1100 has LEDs */
	if ((phydev->phy_id & NXP_PHY_ID_MASK) == NXP_PHY_ID_TJA1100) {
		reg_val = phy_read(phydev, MII_CFG1);
		if (reg_val < 0)
			goto phy_read_error;

		if (reg_val & TJA1100_CFG1_LED_EN) {
			/* mask led mode bits */
			reg_val &= TJA1100_CFG1_LED_MODE;

			switch (reg_val) {
			case CFG1_LED_LINKUP:
				lmode = "LINKUP\n";
				break;
			case CFG1_LED_FRAMEREC:
				lmode = "FRAMEREC\n";
				break;
			case CFG1_LED_SYMERR:
				lmode = "SYMERR\n";
				break;
			case CFG1_LED_CRSSIG:
				lmode = "CRSSIG\n";
				break;
			default:
				lmode = "unknown\n";
				if (verbosity > 1)
					dev_alert(&phydev->PHYDEV_DEV,
					"unknown reg val is [%08x]\n", reg_val);
			}
		}
	}

	/* write result into the buffer */
	return scnprintf(buf, PAGE_SIZE, lmode);

/* error handling */
phy_read_error:
	dev_err(&phydev->PHYDEV_DEV, "read error: %s failed\n", __func__);
	return reg_val;
}

/* This function handles write accesses to the node 'led_cfg' in sysfs
 * Depending on the value written to it, the led mode is configured
 * accordingly.
 */
static ssize_t sysfs_set_led_cfg(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	int err;
	int lmode;
	struct phy_device *phydev = container_of(dev, struct phy_device, PHYDEV_DEV);

	if ((phydev->phy_id & NXP_PHY_ID_MASK) != NXP_PHY_ID_TJA1100)
		goto no_led_error;

	/* parse the buffer */
	err = kstrtoint(buf, 10, &lmode);
	if (err < 0)
		goto phy_parse_error;

	switch (lmode) {
	case 0:
		err = enter_led_mode(phydev, NO_LED_MODE);
		break;
	case 1:
		err = enter_led_mode(phydev, LINKUP_LED_MODE);
		break;
	case 2:
		err = enter_led_mode(phydev, FRAMEREC_LED_MODE);
		break;
	case 3:
		err = enter_led_mode(phydev, SYMERR_LED_MODE);
		break;
	case 4:
		err = enter_led_mode(phydev, CRSSIG_LED_MODE);
		break;
	default:
		break;
	}

	if (err)
		goto phy_lmode_transit_error;

	return count;

/* error handling */
phy_parse_error:
	dev_err(&phydev->PHYDEV_DEV, "parse error: %s failed\n", __func__);
	return err;

phy_lmode_transit_error:
	dev_err(&phydev->PHYDEV_DEV, "lmode error: %s failed\n", __func__);
	return err;

no_led_error:
	dev_info(&phydev->PHYDEV_DEV, "phy has no led support\n");
	return count;
}

/* This function handles read accesses to the node 'link_status' in sysfs
 * Depending on current link status of the phy, the node reads
 * 'up' or 'down'
 */
static ssize_t sysfs_get_link_status(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	int linkup;
	struct phy_device *phydev = container_of(dev, struct phy_device, PHYDEV_DEV);

	linkup = get_link_status(phydev);

	/* write result into the buffer */
	return scnprintf(buf, PAGE_SIZE, "%s\n", linkup ? "up" : "down");
}

/* This function handles read accesses to the node 'wakeup_cfg' in sysfs
 * Reading the node returns the current status of the bits
 * FWDPHYLOC, REMWUPHY, LOCWUPHY, FWDPHYREM
 */
static ssize_t sysfs_get_wakeup_cfg(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	int reg_val;
	int fwdphyloc_en, remwuphy_en, locwuphy_en, fwdphyrem_en;
	struct phy_device *phydev = container_of(dev, struct phy_device, PHYDEV_DEV);

	if ((phydev->phy_id & NXP_PHY_ID_MASK) == NXP_PHY_ID_TJA1102P0 ||
	    (phydev->phy_id & NXP_PHY_ID_MASK) == NXP_PHY_ID_TJA1102P1 ||
	    (phydev->phy_id & NXP_PHY_ID_MASK) == NXP_PHY_ID_TJA1101) {
		reg_val = phy_read(phydev, MII_CFG1);
		if (reg_val < 0)
			goto phy_read_error;

		fwdphyloc_en = 0;
		remwuphy_en = 0;
		locwuphy_en = 0;
		fwdphyrem_en = 0;

		if (reg_val & TJA1102_CFG1_FWDPHYLOC)
			fwdphyloc_en = 1;
		if (reg_val & CFG1_REMWUPHY)
			remwuphy_en = 1;
		if (reg_val & CFG1_LOCWUPHY)
			locwuphy_en = 1;
		if (reg_val & CFG1_FWDPHYREM)
			fwdphyrem_en = 1;
	} else if ((phydev->phy_id & NXP_PHY_ID_MASK) == NXP_PHY_ID_TJA1100) {
		remwuphy_en = 1;	/* not configurable, always enabled */
		fwdphyloc_en = 0;	/* not supported */

		/* The status LED and WAKE input share a pin, so ultimately
		 * configuration depends on the hardware setup.
		 * If LED is disabled, we assume the pin is used for WAKE.
		 * In this case, the phy wakes up upon local wakeup event
		 * via the WAKE pin and also forwards it.
		 */
		reg_val = phy_read(phydev, MII_CFG1);
		if (reg_val < 0)
			goto phy_read_error;

		if (reg_val & TJA1100_CFG1_LED_EN) {
			locwuphy_en = 0;
			fwdphyrem_en = 0;
		} else {
			locwuphy_en = 1;
			fwdphyrem_en = 1;
		}
	} else {
		goto unsupported_phy_error;
	}

	/* write result into the buffer */
	return scnprintf(buf, PAGE_SIZE,
	"fwdphyloc[%s], remwuphy[%s], locwuphy[%s], fwdphyrem[%s]\n",
	(fwdphyloc_en ? "on" : "off"),
	(remwuphy_en ? "on" : "off"),
	(locwuphy_en ? "on" : "off"),
	(fwdphyrem_en ? "on" : "off"));

/* error handling */
phy_read_error:
	dev_err(&phydev->PHYDEV_DEV, "read error: %s failed\n", __func__);
	return reg_val;

unsupported_phy_error:
	dev_err(&phydev->PHYDEV_DEV, "unsupported phy, %s failed\n", __func__);
	return -1;
}

/* This function handles write accesses to the node 'wakeup_cfg' in sysfs
 * Depending on the hexadecimal value written, the bits
 * FWDPHYLOC, REMWUPHY, LOCWUPHY, FWDPHYREM are configured
 */
static ssize_t sysfs_set_wakeup_cfg(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int err, reg_val, reg_mask, wakeup_cfg;
	struct phy_device *phydev = container_of(dev, struct phy_device, PHYDEV_DEV);

	/* parse the buffer */
	err = kstrtoint(buf, 16, &wakeup_cfg);
	if (err < 0)
		goto phy_parse_error;

	if ((phydev->phy_id & NXP_PHY_ID_MASK) == NXP_PHY_ID_TJA1102P0 ||
	    (phydev->phy_id & NXP_PHY_ID_MASK) == NXP_PHY_ID_TJA1102P1 ||
	    (phydev->phy_id & NXP_PHY_ID_MASK) == NXP_PHY_ID_TJA1101) {
		reg_val = 0;

		/* the first 4 bits of the supplied hexadecimal value
		 * are interpreted as the wakeup configuration
		 */
		if (wakeup_cfg & SYSFS_FWDPHYLOC)
			reg_val |= TJA1102_CFG1_FWDPHYLOC;
		if (wakeup_cfg & SYSFS_REMWUPHY)
			reg_val |= CFG1_REMWUPHY;
		if (wakeup_cfg & SYSFS_LOCWUPHY)
			reg_val |= CFG1_LOCWUPHY;
		if (wakeup_cfg & SYSFS_FWDPHYREM)
			reg_val |= CFG1_FWDPHYREM;

		reg_mask = (TJA1102_CFG1_FWDPHYLOC | CFG1_REMWUPHY |
			    CFG1_LOCWUPHY | CFG1_FWDPHYREM);

		err = phy_configure_bits(phydev, MII_CFG1, reg_mask, reg_val);
		if (err < 0)
			goto phy_configure_error;
	} else if ((phydev->phy_id & NXP_PHY_ID_MASK) == NXP_PHY_ID_TJA1100) {
		/* FWDPHYLOC MUST be off
		 * REMWUPHY MUST be on
		 * only LOCWUPHY and FWDPHYREM are configurable
		 * Possible configurations:
		 * - BOTH enabled (then led MUST be off)
		 * - BOTH disabled (then led CAN be on)
		 * all other configurations are invalid.
		 *
		 * Therefore valid values to write to sysfs are:
		 * - 2 (LOCWUPHY and FWDPHYREM off)
		 * - E (LOCWUPHY and FWDPHYREM on)
		 */
		if (((wakeup_cfg & SYSFS_LOCWUPHY) !=
		     (wakeup_cfg & SYSFS_FWDPHYREM)) ||
		    wakeup_cfg & SYSFS_FWDPHYLOC ||
		    !(wakeup_cfg & SYSFS_REMWUPHY)) {
			dev_alert(&phydev->PHYDEV_DEV, "Invalid configuration\n");
		} else if (wakeup_cfg & SYSFS_LOCWUPHY &&
			   wakeup_cfg & SYSFS_FWDPHYREM) {
			err = enter_led_mode(phydev, NO_LED_MODE);
			if (err)
				goto phy_lmode_transit_error;
		}
	}

	return count;

/* error handling */
phy_parse_error:
	dev_err(&phydev->PHYDEV_DEV, "parse error: %s failed\n", __func__);
	return err;

phy_configure_error:
	dev_err(&phydev->PHYDEV_DEV, "phy r/w error: %s failed\n", __func__);
	return err;

phy_lmode_transit_error:
	dev_err(&phydev->PHYDEV_DEV, "lmode error: %s failed\n", __func__);
	return err;
}

/* This function handles read accesses to the node 'snr_wlimit_cfg' in sysfs.
 * Reading the node returns the current snr warning limit.
 */
static ssize_t sysfs_get_snr_wlimit_cfg(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int reg_val;
	char *snr_limit;
	struct phy_device *phydev = container_of(dev, struct phy_device, PHYDEV_DEV);

	reg_val = phy_read(phydev, MII_CFG2);
	if (reg_val < 0)
		goto phy_read_error;

	/* mask snr wlimit bits */
	reg_val &= CFG2_SNR_WLIMIT;

	switch (reg_val) {
	case SNR_CLASS_NONE:
		snr_limit = "no fail limit\n";
		break;
	case SNR_CLASS_A:
		snr_limit = "CLASS_A\n";
		break;
	case SNR_CLASS_B:
		snr_limit = "CLASS_B\n";
		break;
	case SNR_CLASS_C:
		snr_limit = "CLASS_C\n";
		break;
	case SNR_CLASS_D:
		snr_limit = "CLASS_D\n";
		break;
	case SNR_CLASS_E:
		snr_limit = "CLASS_E\n";
		break;
	case SNR_CLASS_F:
		snr_limit = "CLASS_F\n";
		break;
	case SNR_CLASS_G:
		snr_limit = "CLASS_G\n";
		break;
	default:
		snr_limit = "unknown\n";
	}

	/* write result into the buffer */
	return scnprintf(buf, PAGE_SIZE, snr_limit);

/* error handling */
phy_read_error:
	dev_err(&phydev->PHYDEV_DEV, "read error: %s failed\n", __func__);
	return reg_val;
}

/* This function handles write accesses to the node 'snr_wlimit_cfg' in sysfs
 * Depending on the value written to it, the snr warning limit is configured
 * accordingly.
 */
static ssize_t sysfs_set_snr_wlimit_cfg(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int err, snr_limit, reg_val;
	struct phy_device *phydev = container_of(dev, struct phy_device, PHYDEV_DEV);

	/* parse the buffer */
	err = kstrtoint(buf, 10, &snr_limit);
	if (err < 0)
		goto phy_parse_error;

	switch (snr_limit) {
	case 0:
		reg_val = SNR_CLASS_NONE;
		break;
	case 1:
		reg_val = SNR_CLASS_A;
		break;
	case 2:
		reg_val = SNR_CLASS_B;
		break;
	case 3:
		reg_val = SNR_CLASS_C;
		break;
	case 4:
		reg_val = SNR_CLASS_D;
		break;
	case 5:
		reg_val = SNR_CLASS_E;
		break;
	case 6:
		reg_val = SNR_CLASS_F;
		break;
	case 7:
		reg_val = SNR_CLASS_G;
		break;
	default:
		reg_val = -1;
		break;
	}

	if (reg_val != -1) {
		err = phy_configure_bits(phydev, MII_CFG2,
					 CFG2_SNR_WLIMIT, reg_val);
		if (err)
			goto phy_configure_error;
	}

	return count;

/* error handling */
phy_parse_error:
	dev_err(&phydev->PHYDEV_DEV, "parse error: %s failed\n", __func__);
	return err;

phy_configure_error:
	dev_err(&phydev->PHYDEV_DEV,
		"phy r/w error: %s failed\n", __func__);
	return err;
}

/* r/w access for everyone */
static DEVICE_ATTR(master_cfg, S_IWUSR | S_IRUSR,
		   sysfs_get_master_cfg, sysfs_set_master_cfg);
static DEVICE_ATTR(power_cfg, S_IWUSR | S_IRUSR,
		   sysfs_get_power_cfg, sysfs_set_power_cfg);
static DEVICE_ATTR(loopback_cfg, S_IWUSR | S_IRUSR,
		   sysfs_get_loopback_cfg, sysfs_set_loopback_cfg);
static DEVICE_ATTR(cable_test, S_IRUSR, sysfs_get_cable_test, NULL);
static DEVICE_ATTR(test_mode, S_IWUSR | S_IRUSR,
		   sysfs_get_test_mode, sysfs_set_test_mode);
static DEVICE_ATTR(led_cfg, S_IWUSR | S_IRUSR,
		   sysfs_get_led_cfg, sysfs_set_led_cfg);
static DEVICE_ATTR(link_status, S_IRUSR, sysfs_get_link_status, NULL);
static DEVICE_ATTR(wakeup_cfg, S_IWUSR | S_IRUSR,
		   sysfs_get_wakeup_cfg, sysfs_set_wakeup_cfg);
static DEVICE_ATTR(snr_wlimit_cfg, S_IWUSR | S_IRUSR,
		   sysfs_get_snr_wlimit_cfg, sysfs_set_snr_wlimit_cfg);

static struct attribute *nxp_sysfs_entries[] = {
	&dev_attr_master_cfg.attr,
	&dev_attr_power_cfg.attr,
	&dev_attr_loopback_cfg.attr,
	&dev_attr_cable_test.attr,
	&dev_attr_test_mode.attr,
	&dev_attr_led_cfg.attr,
	&dev_attr_link_status.attr,
	&dev_attr_wakeup_cfg.attr,
	&dev_attr_snr_wlimit_cfg.attr,
	NULL
};

static struct attribute_group nxp_attribute_group = {
	.name = "configuration",
	.attrs = nxp_sysfs_entries,
};

/* helper function, configures a register of phydev
 *
 * The function sets the bit of register reg_name,
 * defined by bit_mask to 0 if (bit_value == 0), else to 1
 * @return	0 if configuration completed, <0 if read/write
 *		error occurred
 */
static inline int phy_configure_bit(struct phy_device *phydev, int reg_name,
				    int bit_mask, int bit_value)
{
	int reg_val, err;

	if (verbosity > 2)
		dev_alert(&phydev->PHYDEV_DEV, "%s bit on mask [%08x] of reg [%d] of phy %x\n",
		(bit_value?"enabling":"disabling"), bit_mask, reg_name, phydev->PHYDEV_ADDR);

	reg_val = phy_read(phydev, reg_name);
	if (reg_val < 0)
		goto phy_read_error;

	if (bit_value)
		reg_val |= bit_mask;
	else
		reg_val &= ~bit_mask;

	err = phy_write(phydev, reg_name, reg_val);
	if (err < 0)
		goto phy_write_error;

	return 0;

/* error handling */
phy_read_error:
	dev_err(&phydev->PHYDEV_DEV, "read error: phy %s failed\n", __func__);
	return reg_val;

phy_write_error:
	dev_err(&phydev->PHYDEV_DEV, "write error: phy %s failed\n", __func__);
	return err;
}

/* helper function, configures a register of phydev
 *
 * The function sets the bits of register reg_name,
 * defined by bit_mask to bit_value
 * @return	0 if configuration completed, <0 if read/write
 *		error occurred
 */
static inline int phy_configure_bits(struct phy_device *phydev, int reg_name,
				     int bit_mask, int bit_value)
{
	int reg_val, err;

	if (verbosity > 2)
		dev_alert(&phydev->PHYDEV_DEV, "set mask [%08x] of reg [%d] of phy %x to value [%08x]\n",
		bit_mask, reg_name, phydev->PHYDEV_ADDR, bit_value);

	reg_val = phy_read(phydev, reg_name);
	if (reg_val < 0)
		goto phy_read_error;

	reg_val &= ~bit_mask;
	reg_val |= bit_value;

	err = phy_write(phydev, reg_name, reg_val);
	if (err < 0)
		goto phy_write_error;

	return 0;

/* error handling */
phy_read_error:
	dev_err(&phydev->PHYDEV_DEV, "read error: phy %s failed\n", __func__);
	return reg_val;

phy_write_error:
	dev_err(&phydev->PHYDEV_DEV, "write error: phy %s failed\n", __func__);
	return err;
}

#if defined(CONFIG_STANDALONE_PHY) && defined(CONFIG_POLL)
static struct class *bus_class_from_net_device(struct net_device *net_device,
					       const char *required_name)
{
	struct class *bus_class;

	if (!net_device ||
	    !net_device->phydev ||
	    !net_device->phydev->PHYDEV_BUS ||
	    !net_device->phydev->PHYDEV_BUS->dev.class ||
	    !net_device->phydev->PHYDEV_BUS->dev.class->name)
		return NULL;

	bus_class = net_device->phydev->PHYDEV_BUS->dev.class;
	if (strcmp(bus_class->name, required_name) != 0)
		return NULL;

	return bus_class;
}

static int mdio_bus_name_matches(struct device *found_device,
				 const void *desired_name)
{
	struct mii_bus *mdio_bus;

	/* Since we know 'found_dev' belongs to a class with the name
	   'mdio_bus', we assume it is a member of a 'struct mii_bus',
	 and therefore it is safe to call container_of */
	mdio_bus = container_of(found_device, struct mii_bus, dev);

	/* Double check that this is indeed a 'struct mii_bus'. If it is,
	 it's state should be MDIO_REGISTERED at this point. If it is not, it is
	 either not a 'struct mii_bus', either it is in an undesired state. */
	if (mdio_bus->state != MDIOBUS_REGISTERED)
		return 0;

	if (strcmp(mdio_bus->name, (char *)desired_name) == 0)
		return 1;
	return 0;
}

static struct mii_bus *find_mdio_bus_by_name(const char *name,
					     struct class *mdio_bus_class)
{
	struct device *found_device;

	found_device = class_find_device(mdio_bus_class,
					 NULL,
					 (void *)name,
					 mdio_bus_name_matches);
	if (found_device)
		return container_of(found_device, struct mii_bus, dev);
	else
		return NULL;
}

/* helper function, check if given phy id belongs to a nxp phy
 *
 * @return	0 if not an nxp phy, != 0 else
 */
static bool is_nxp_phy(int phy_id)
{
	return ((phy_id & NXP_PHY_ID_MASK) == NXP_PHY_ID_TJA1100 ||
		(phy_id & NXP_PHY_ID_MASK) == NXP_PHY_ID_TJA1101 ||
		(phy_id & NXP_PHY_ID_MASK) == NXP_PHY_ID_TJA1102P0 ||
		(phy_id & NXP_PHY_ID_MASK) == NXP_PHY_ID_TJA1102P1 ||
		(phy_id & NXP_PHY_ID_MASK) == NXP_PHY_ID_TJA1102S);

}

/* traverse the phy_map of the given mdio_bus, and manipulate any phys found
 * that are found according to the value of the event, ie.
 * - start (resume) on NETDEV_UP
 * - stop (suspend) on NETDEV_GOING_DOWN
 */
static void mdio_netdev_change_event(struct mii_bus *mdio_bus, int event)
{
	/* normally on NETDEV_GOING_DOWN the kernel calls ndo_stop()
	 * of the eth controller, which stops and disconnects the one phy
	 * that is associated with the ethernet controller
	 * [see fec_enet_close() in fec_main.c l 2740].
	 * We need to do this manually for every NXP phy,
	 * however we do not (necessarily) have an attached_dev, so phy_detach,
	 * which is called by phy_disconnect(), would crash
	 */
	int phy_addr;
	struct phy_device *phydev;

	for (phy_addr = 0; phy_addr < PHY_MAX_ADDR; phy_addr++) {
		phydev = GET_PHYDEV(mdio_bus, phy_addr);
		if (!phydev)
			continue;

		if (!is_nxp_phy(phydev->phy_id) || !phydev->priv)
			continue;

		if (event == NETDEV_GOING_DOWN) {
			/* stop polling,
			 * as mdio bus will become unavailable as soon as
			 * fec_runtime_suspend() (fec_main.c l4801) is called
			 */
			if (!no_poll)
				stop_polling(phydev);

			/* sets state to PHY_HALTED */
			phy_stop(phydev);
		} else if (event == NETDEV_UP) {
			/* updates the phy state and resumes,
			 * if state previously was PHY_HALTED
			 */
			phy_start(phydev);

			if (!no_poll)
				start_polling(phydev);
		}
	}
}

/* callback, called whenever a netdev changes its state.
 *
 * Handles only NETDEV_GOING_DOWN and NETDEV_UP events of interface called
 * MDIO_INTERFACE_NAME. Phys on the mdio bus "fec_enet_mii_bus"
 * are stopped (suspended) and started (resumed) accordingly.
 *
 * @return	NOTIFY_DONE
 */
static int netdev_state_change_event(struct notifier_block *unused,
				     unsigned long event, void *ptr)
{
	struct net_device *dev = netdev_notifier_info_to_dev(ptr);
	struct mii_bus *mdio_bus;
	struct class *bus_class;
	struct net_device *net_device;

	/* currently the eth0 interface controlls the mdio bus.
	 * However as CONFIG_FIXED_PHY is configured,
	 * eth0 is associated with "Fixed MDIO Bus", but the phydev
	 * is associated with "fec_enet_mii_bus". If eth0 goes down,
	 * only devices on "Fixed MDIO Bus" are notified (ie removed).
	 * We need to manually listen to eth0 events
	 * stops the phy and the polling
	 */
	if (event != NETDEV_GOING_DOWN && event != NETDEV_UP)
		goto skip;

	if (strcmp(dev->name, MDIO_INTERFACE_NAME) != 0)
		goto skip;

	net_device = first_net_device(&init_net);
	do {
		bus_class = bus_class_from_net_device(net_device, "mdio_bus");
		if (!bus_class)
			continue;
		mdio_bus = find_mdio_bus_by_name(MII_BUS_NAME, bus_class);
		if (!mdio_bus)
			continue;

		if (verbosity > 0)
			pr_alert("NXP PHY: received event [%lx] for [%s]: Notifying phys on bus [%s]\n",
			event, dev->name, mdio_bus->name);

		mdio_netdev_change_event(mdio_bus, event);
	} while ((net_device = next_net_device(net_device)));

skip:
	return NOTIFY_DONE;
}

/* netdev notification infrastructure */
struct notifier_block netdev_notifier = {
	.notifier_call = netdev_state_change_event
};
#endif

static struct phy_driver nxp_drivers[] = {
	{
	.phy_id = NXP_PHY_ID_TJA1100,
	.name = "TJA1100",
	.phy_id_mask = NXP_PHY_ID_MASK,
	.features = (SUPPORTED_TP | SUPPORTED_MII | SUPPORTED_100BASET1_FULL),
	.flags = 0,
	.probe = &nxp_probe,
	.remove = &nxp_remove,
	.config_init = &nxp_config_init,
	.config_aneg = &nxp_config_aneg,
	.read_status = &nxp_read_status,
	.resume = &nxp_resume,
	.suspend = &nxp_suspend,
	.config_intr = &nxp_config_intr,
	.ack_interrupt = &nxp_ack_interrupt,
	.did_interrupt = &nxp_did_interrupt,
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,5,0)
	.driver = {.owner = THIS_MODULE},
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,13,1)
	.set_loopback = nxp_set_loopback,
#endif
	},
	{
	.phy_id = NXP_PHY_ID_TJA1102P0,
	.name = "TJA1102_p0",
	.phy_id_mask = NXP_PHY_ID_MASK,
	.features = (SUPPORTED_TP | SUPPORTED_MII | SUPPORTED_100BASET1_FULL),
	.flags = 0,
	.probe = &nxp_probe,
	.remove = &nxp_remove,
	.config_init = &nxp_config_init,
	.config_aneg = &nxp_config_aneg,
	.read_status = &nxp_read_status,
	.resume = &nxp_resume,
	.suspend = &nxp_suspend,
	.config_intr = &nxp_config_intr,
	.ack_interrupt = &nxp_ack_interrupt,
	.did_interrupt = &nxp_did_interrupt,
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,5,0)
	.driver = {.owner = THIS_MODULE},
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,13,1)
	.set_loopback = nxp_set_loopback,
#endif
	},
	{
	.phy_id = NXP_PHY_ID_TJA1101,
	.name = "TJA1101",
	.phy_id_mask = NXP_PHY_ID_MASK,
	.features = (SUPPORTED_TP | SUPPORTED_MII | SUPPORTED_100BASET1_FULL),
	.flags = 0,
	.probe = &nxp_probe,
	.remove = &nxp_remove,
	.config_init = &nxp_config_init,
	.config_aneg = &nxp_config_aneg,
	.read_status = &nxp_read_status,
	.resume = &nxp_resume,
	.suspend = &nxp_suspend,
	.config_intr = &nxp_config_intr,
	.ack_interrupt = &nxp_ack_interrupt,
	.did_interrupt = &nxp_did_interrupt,
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,5,0)
	.driver = {.owner = THIS_MODULE},
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,13,1)
	.set_loopback = nxp_set_loopback,
#endif
	},
	{
	.phy_id = NXP_PHY_ID_TJA1102S,
	.name = "TJA1102S",
	.phy_id_mask = NXP_PHY_ID_MASK,
	.features = (SUPPORTED_TP | SUPPORTED_MII | SUPPORTED_100BASET1_FULL),
	.flags = 0,
	.probe = &nxp_probe,
	.remove = &nxp_remove,
	.config_init = &nxp_config_init,
	.config_aneg = &nxp_config_aneg,
	.read_status = &nxp_read_status,
	.resume = &nxp_resume,
	.suspend = &nxp_suspend,
	.config_intr = &nxp_config_intr,
	.ack_interrupt = &nxp_ack_interrupt,
	.did_interrupt = &nxp_did_interrupt,
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,5,0)
	.driver = {.owner = THIS_MODULE},
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,13,1)
	.set_loopback = nxp_set_loopback,
#endif
	}
#ifdef CONFIG_TJA1102_FIX
	, {
	.phy_id = NXP_PHY_ID_TJA1102P1,
	.name = "TJA1102_p1",
	.phy_id_mask = NXP_PHY_ID_MASK,
	.features = (SUPPORTED_TP | SUPPORTED_MII | SUPPORTED_100BASET1_FULL),
	.flags = 0,
	.probe = &nxp_probe,
	.remove = &nxp_remove,
	.config_init = &nxp_config_init,
	.config_aneg = &nxp_config_aneg,
	.read_status = &nxp_read_status,
	.resume = &nxp_resume,
	.suspend = &nxp_suspend,
	.config_intr = &nxp_config_intr,
	.ack_interrupt = &nxp_ack_interrupt,
	.did_interrupt = &nxp_did_interrupt,
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,5,0)
	.driver = {.owner = THIS_MODULE},
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,13,1)
	.set_loopback = nxp_set_loopback,
#endif
	}
#endif
};

/* module init function */
static int __init nxp_init(void)
{
	int err;

	pr_alert("NXP PHY: loading NXP PHY driver: [%s, polling %s]\n",
		 (managed_mode ? "managed mode" : "autonomous mode"),
		 (no_poll ? "disabled" : "enabled"));

#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,5,0)
	err = phy_drivers_register(nxp_drivers, ARRAY_SIZE(nxp_drivers));
#else
	err = phy_drivers_register(nxp_drivers, ARRAY_SIZE(nxp_drivers), THIS_MODULE);
#endif

	if (err)
		goto drv_registration_error;

#if defined(CONFIG_STANDALONE_PHY) && defined(CONFIG_POLL)
	if (!no_poll) {
		err = register_netdevice_notifier(&netdev_notifier);
		if (err)
			goto notification_registration_error;
	}
#endif

	return 0;

/* error handling */
drv_registration_error:
	pr_err("NXP PHY: driver registration failed\n");
	return err;

#if defined(CONFIG_STANDALONE_PHY) && defined(CONFIG_POLL)
notification_registration_error:
	pr_err("NXP PHY: could not register notification handler\n");
	unregister_netdevice_notifier(&netdev_notifier);
	return err;
#endif
}

module_init(nxp_init);

/* module exit function */
static void __exit nxp_exit(void)
{
	pr_alert("NXP PHY: unloading NXP PHY driver\n");
#if defined(CONFIG_STANDALONE_PHY) && defined(CONFIG_POLL)
	if (!no_poll)
		unregister_netdevice_notifier(&netdev_notifier);
#endif
	phy_drivers_unregister(nxp_drivers, ARRAY_SIZE(nxp_drivers));
}

module_exit(nxp_exit);

/* use module device table for hotplugging support */
static struct mdio_device_id __maybe_unused nxp_tbl[] = {
	{NXP_PHY_ID_TJA1100, NXP_PHY_ID_MASK},
	{NXP_PHY_ID_TJA1102P0, NXP_PHY_ID_MASK},
	{NXP_PHY_ID_TJA1102S, NXP_PHY_ID_MASK},
	{}
};

MODULE_DEVICE_TABLE(mdio, nxp_tbl);

MODULE_DESCRIPTION("NXP PHY driver");
MODULE_AUTHOR("Marco Hartmann");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.5");
