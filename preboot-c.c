/**
 * core.h - DesignWare USB3 DRD Core Header
 * linux commit 7bc5a6ba369217e0137833f5955cf0b0f08b0712 before
 * the switch to GPLv2 only
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com
 *
 * Authors: Felipe Balbi <balbi@ti.com>,
 *	    Sebastian Andrzej Siewior <bigeasy@linutronix.de>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2, as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __DRIVERS_USB_DWC3_CORE_H
#define __DRIVERS_USB_DWC3_CORE_H

/* Global constants */
#define DWC3_EP0_BOUNCE_SIZE    512
#define DWC3_ENDPOINTS_NUM      32
#define DWC3_XHCI_RESOURCES_NUM 2

#define DWC3_EVENT_SIZE         4  /* bytes */
#define DWC3_EVENT_MAX_NUM      64 /* 2 events/endpoint */
#define DWC3_EVENT_BUFFERS_SIZE (DWC3_EVENT_SIZE * DWC3_EVENT_MAX_NUM)
#define DWC3_EVENT_TYPE_MASK    0xfe

#define DWC3_EVENT_TYPE_DEV    0
#define DWC3_EVENT_TYPE_CARKIT 3
#define DWC3_EVENT_TYPE_I2C    4

#define DWC3_DEVICE_EVENT_DISCONNECT         0
#define DWC3_DEVICE_EVENT_RESET              1
#define DWC3_DEVICE_EVENT_CONNECT_DONE       2
#define DWC3_DEVICE_EVENT_LINK_STATUS_CHANGE 3
#define DWC3_DEVICE_EVENT_WAKEUP             4
#define DWC3_DEVICE_EVENT_HIBER_REQ          5
#define DWC3_DEVICE_EVENT_EOPF               6
#define DWC3_DEVICE_EVENT_SOF                7
#define DWC3_DEVICE_EVENT_ERRATIC_ERROR      9
#define DWC3_DEVICE_EVENT_CMD_CMPL           10
#define DWC3_DEVICE_EVENT_OVERFLOW           11

#define DWC3_GEVNTCOUNT_MASK 0xfffc
#define DWC3_GSNPSID_MASK    0xffff0000
#define DWC3_GSNPSREV_MASK   0xffff

/* DWC3 registers memory space boundries */
#define DWC3_XHCI_REGS_START    0x0
#define DWC3_XHCI_REGS_END      0x7fff
#define DWC3_GLOBALS_REGS_START 0xc100
#define DWC3_GLOBALS_REGS_END   0xc6ff
#define DWC3_DEVICE_REGS_START  0xc700
#define DWC3_DEVICE_REGS_END    0xcbff
#define DWC3_OTG_REGS_START     0xcc00
#define DWC3_OTG_REGS_END       0xccff

/* Global Registers */
#define DWC3_GSBUSCFG0     0xc100
#define DWC3_GSBUSCFG1     0xc104
#define DWC3_GTXTHRCFG     0xc108
#define DWC3_GRXTHRCFG     0xc10c
#define DWC3_GCTL          0xc110
#define DWC3_GEVTEN        0xc114
#define DWC3_GSTS          0xc118
#define DWC3_GSNPSID       0xc120
#define DWC3_GGPIO         0xc124
#define DWC3_GUID          0xc128
#define DWC3_GUCTL         0xc12c
#define DWC3_GBUSERRADDR0  0xc130
#define DWC3_GBUSERRADDR1  0xc134
#define DWC3_GPRTBIMAP0    0xc138
#define DWC3_GPRTBIMAP1    0xc13c
#define DWC3_GHWPARAMS0    0xc140
#define DWC3_GHWPARAMS1    0xc144
#define DWC3_GHWPARAMS2    0xc148
#define DWC3_GHWPARAMS3    0xc14c
#define DWC3_GHWPARAMS4    0xc150
#define DWC3_GHWPARAMS5    0xc154
#define DWC3_GHWPARAMS6    0xc158
#define DWC3_GHWPARAMS7    0xc15c
#define DWC3_GDBGFIFOSPACE 0xc160
#define DWC3_GDBGLTSSM     0xc164
#define DWC3_GPRTBIMAP_HS0 0xc180
#define DWC3_GPRTBIMAP_HS1 0xc184
#define DWC3_GPRTBIMAP_FS0 0xc188
#define DWC3_GPRTBIMAP_FS1 0xc18c

#define DWC3_GUSB2PHYCFG(n) (0xc200 + (n * 0x04))
#define DWC3_GUSB2I2CCTL(n) (0xc240 + (n * 0x04))

#define DWC3_GUSB2PHYACC(n) (0xc280 + (n * 0x04))

#define DWC3_GUSB3PIPECTL(n) (0xc2c0 + (n * 0x04))

#define DWC3_GTXFIFOSIZ(n) (0xc300 + (n * 0x04))
#define DWC3_GRXFIFOSIZ(n) (0xc380 + (n * 0x04))

#define DWC3_GEVNTADRLO(n) (0xc400 + (n * 0x10))
#define DWC3_GEVNTADRHI(n) (0xc404 + (n * 0x10))
#define DWC3_GEVNTSIZ(n)   (0xc408 + (n * 0x10))
#define DWC3_GEVNTCOUNT(n) (0xc40c + (n * 0x10))

#define DWC3_GHWPARAMS8 0xc600

/* Device Registers */
#define DWC3_DCFG          0xc700
#define DWC3_DCTL          0xc704
#define DWC3_DEVTEN        0xc708
#define DWC3_DSTS          0xc70c
#define DWC3_DGCMDPAR      0xc710
#define DWC3_DGCMD         0xc714
#define DWC3_DALEPENA      0xc720
#define DWC3_DEPCMDPAR2(n) (0xc800 + (n * 0x10))
#define DWC3_DEPCMDPAR1(n) (0xc804 + (n * 0x10))
#define DWC3_DEPCMDPAR0(n) (0xc808 + (n * 0x10))
#define DWC3_DEPCMD(n)     (0xc80c + (n * 0x10))

/* OTG Registers */
#define DWC3_OCFG   0xcc00
#define DWC3_OCTL   0xcc04
#define DWC3_OEVT   0xcc08
#define DWC3_OEVTEN 0xcc0C
#define DWC3_OSTS   0xcc10

/* Bit fields */

/* Global Configuration Register */
#define DWC3_GCTL_PWRDNSCALE(n) ((n) << 19)
#define DWC3_GCTL_U2RSTECN      (1 << 16)
#define DWC3_GCTL_RAMCLKSEL(x)  (((x)&DWC3_GCTL_CLK_MASK) << 6)
#define DWC3_GCTL_CLK_BUS       (0)
#define DWC3_GCTL_CLK_PIPE      (1)
#define DWC3_GCTL_CLK_PIPEHALF  (2)
#define DWC3_GCTL_CLK_MASK      (3)

#define DWC3_GCTL_PRTCAP(n)     (((n) & (3 << 12)) >> 12)
#define DWC3_GCTL_PRTCAPDIR(n)  ((n) << 12)
#define DWC3_GCTL_PRTCAP_HOST   1
#define DWC3_GCTL_PRTCAP_DEVICE 2
#define DWC3_GCTL_PRTCAP_OTG    3

#define DWC3_GCTL_CORESOFTRESET    (1 << 11)
#define DWC3_GCTL_SCALEDOWN(n)     ((n) << 4)
#define DWC3_GCTL_SCALEDOWN_MASK   DWC3_GCTL_SCALEDOWN(3)
#define DWC3_GCTL_DISSCRAMBLE      (1 << 3)
#define DWC3_GCTL_GBLHIBERNATIONEN (1 << 1)
#define DWC3_GCTL_DSBLCLKGTNG      (1 << 0)

/* Global USB2 PHY Configuration Register */
#define DWC3_GUSB2PHYCFG_PHYSOFTRST (1 << 31)
#define DWC3_GUSB2PHYCFG_SUSPHY     (1 << 6)

/* Global USB3 PIPE Control Register */
#define DWC3_GUSB3PIPECTL_PHYSOFTRST (1 << 31)
#define DWC3_GUSB3PIPECTL_SUSPHY     (1 << 17)

/* Global TX Fifo Size Register */
#define DWC3_GTXFIFOSIZ_TXFDEF(n)    ((n)&0xffff)
#define DWC3_GTXFIFOSIZ_TXFSTADDR(n) ((n)&0xffff0000)

/* Global HWPARAMS1 Register */
#define DWC3_GHWPARAMS1_EN_PWROPT(n)  (((n) & (3 << 24)) >> 24)
#define DWC3_GHWPARAMS1_EN_PWROPT_NO  0
#define DWC3_GHWPARAMS1_EN_PWROPT_CLK 1
#define DWC3_GHWPARAMS1_EN_PWROPT_HIB 2
#define DWC3_GHWPARAMS1_PWROPT(n)     ((n) << 24)
#define DWC3_GHWPARAMS1_PWROPT_MASK   DWC3_GHWPARAMS1_PWROPT(3)

/* Global HWPARAMS4 Register */
#define DWC3_GHWPARAMS4_HIBER_SCRATCHBUFS(n) (((n) & (0x0f << 13)) >> 13)
#define DWC3_MAX_HIBER_SCRATCHBUFS           15

/* Device Configuration Register */
#define DWC3_DCFG_LPM_CAP       (1 << 22)
#define DWC3_DCFG_DEVADDR(addr) ((addr) << 3)
#define DWC3_DCFG_DEVADDR_MASK  DWC3_DCFG_DEVADDR(0x7f)

#define DWC3_DCFG_SPEED_MASK (7 << 0)
#define DWC3_DCFG_SUPERSPEED (4 << 0)
#define DWC3_DCFG_HIGHSPEED  (0 << 0)
#define DWC3_DCFG_FULLSPEED2 (1 << 0)
#define DWC3_DCFG_LOWSPEED   (2 << 0)
#define DWC3_DCFG_FULLSPEED1 (3 << 0)

#define DWC3_DCFG_LPM_CAP (1 << 22)

/* Device Control Register */
#define DWC3_DCTL_RUN_STOP (1 << 31)
#define DWC3_DCTL_CSFTRST  (1 << 30)
#define DWC3_DCTL_LSFTRST  (1 << 29)

#define DWC3_DCTL_HIRD_THRES_MASK (0x1f << 24)
#define DWC3_DCTL_HIRD_THRES(n)   ((n) << 24)

#define DWC3_DCTL_APPL1RES (1 << 23)

/* These apply for core versions 1.87a and earlier */
#define DWC3_DCTL_TRGTULST_MASK     (0x0f << 17)
#define DWC3_DCTL_TRGTULST(n)       ((n) << 17)
#define DWC3_DCTL_TRGTULST_U2       (DWC3_DCTL_TRGTULST(2))
#define DWC3_DCTL_TRGTULST_U3       (DWC3_DCTL_TRGTULST(3))
#define DWC3_DCTL_TRGTULST_SS_DIS   (DWC3_DCTL_TRGTULST(4))
#define DWC3_DCTL_TRGTULST_RX_DET   (DWC3_DCTL_TRGTULST(5))
#define DWC3_DCTL_TRGTULST_SS_INACT (DWC3_DCTL_TRGTULST(6))

/* These apply for core versions 1.94a and later */
#define DWC3_DCTL_KEEP_CONNECT (1 << 19)
#define DWC3_DCTL_L1_HIBER_EN  (1 << 18)
#define DWC3_DCTL_CRS          (1 << 17)
#define DWC3_DCTL_CSS          (1 << 16)

#define DWC3_DCTL_INITU2ENA    (1 << 12)
#define DWC3_DCTL_ACCEPTU2ENA  (1 << 11)
#define DWC3_DCTL_INITU1ENA    (1 << 10)
#define DWC3_DCTL_ACCEPTU1ENA  (1 << 9)
#define DWC3_DCTL_TSTCTRL_MASK (0xf << 1)

#define DWC3_DCTL_ULSTCHNGREQ_MASK (0x0f << 5)
#define DWC3_DCTL_ULSTCHNGREQ(n)   (((n) << 5) & DWC3_DCTL_ULSTCHNGREQ_MASK)

#define DWC3_DCTL_ULSTCHNG_NO_ACTION   (DWC3_DCTL_ULSTCHNGREQ(0))
#define DWC3_DCTL_ULSTCHNG_SS_DISABLED (DWC3_DCTL_ULSTCHNGREQ(4))
#define DWC3_DCTL_ULSTCHNG_RX_DETECT   (DWC3_DCTL_ULSTCHNGREQ(5))
#define DWC3_DCTL_ULSTCHNG_SS_INACTIVE (DWC3_DCTL_ULSTCHNGREQ(6))
#define DWC3_DCTL_ULSTCHNG_RECOVERY    (DWC3_DCTL_ULSTCHNGREQ(8))
#define DWC3_DCTL_ULSTCHNG_COMPLIANCE  (DWC3_DCTL_ULSTCHNGREQ(10))
#define DWC3_DCTL_ULSTCHNG_LOOPBACK    (DWC3_DCTL_ULSTCHNGREQ(11))

/* Device Event Enable Register */
#define DWC3_DEVTEN_VNDRDEVTSTRCVEDEN   (1 << 12)
#define DWC3_DEVTEN_EVNTOVERFLOWEN      (1 << 11)
#define DWC3_DEVTEN_CMDCMPLTEN          (1 << 10)
#define DWC3_DEVTEN_ERRTICERREN         (1 << 9)
#define DWC3_DEVTEN_SOFEN               (1 << 7)
#define DWC3_DEVTEN_EOPFEN              (1 << 6)
#define DWC3_DEVTEN_HIBERNATIONREQEVTEN (1 << 5)
#define DWC3_DEVTEN_WKUPEVTEN           (1 << 4)
#define DWC3_DEVTEN_ULSTCNGEN           (1 << 3)
#define DWC3_DEVTEN_CONNECTDONEEN       (1 << 2)
#define DWC3_DEVTEN_USBRSTEN            (1 << 1)
#define DWC3_DEVTEN_DISCONNEVTEN        (1 << 0)

/* Device Status Register */
#define DWC3_DSTS_DCNRD (1 << 29)

/* This applies for core versions 1.87a and earlier */
#define DWC3_DSTS_PWRUPREQ (1 << 24)

/* These apply for core versions 1.94a and later */
#define DWC3_DSTS_RSS (1 << 25)
#define DWC3_DSTS_SSS (1 << 24)

#define DWC3_DSTS_COREIDLE   (1 << 23)
#define DWC3_DSTS_DEVCTRLHLT (1 << 22)

#define DWC3_DSTS_USBLNKST_MASK (0x0f << 18)
#define DWC3_DSTS_USBLNKST(n)   (((n)&DWC3_DSTS_USBLNKST_MASK) >> 18)

#define DWC3_DSTS_RXFIFOEMPTY (1 << 17)

#define DWC3_DSTS_SOFFN_MASK (0x3fff << 3)
#define DWC3_DSTS_SOFFN(n)   (((n)&DWC3_DSTS_SOFFN_MASK) >> 3)

#define DWC3_DSTS_CONNECTSPD (7 << 0)

#define DWC3_DSTS_SUPERSPEED (4 << 0)
#define DWC3_DSTS_HIGHSPEED  (0 << 0)
#define DWC3_DSTS_FULLSPEED2 (1 << 0)
#define DWC3_DSTS_LOWSPEED   (2 << 0)
#define DWC3_DSTS_FULLSPEED1 (3 << 0)

/* Device Generic Command Register */
#define DWC3_DGCMD_SET_LMP          0x01
#define DWC3_DGCMD_SET_PERIODIC_PAR 0x02
#define DWC3_DGCMD_XMIT_FUNCTION    0x03

/* These apply for core versions 1.94a and later */
#define DWC3_DGCMD_SET_SCRATCHPAD_ADDR_LO 0x04
#define DWC3_DGCMD_SET_SCRATCHPAD_ADDR_HI 0x05

#define DWC3_DGCMD_SELECTED_FIFO_FLUSH  0x09
#define DWC3_DGCMD_ALL_FIFO_FLUSH       0x0a
#define DWC3_DGCMD_SET_ENDPOINT_NRDY    0x0c
#define DWC3_DGCMD_RUN_SOC_BUS_LOOPBACK 0x10

#define DWC3_DGCMD_STATUS(n) (((n) >> 15) & 1)
#define DWC3_DGCMD_CMDACT    (1 << 10)
#define DWC3_DGCMD_CMDIOC    (1 << 8)

/* Device Generic Command Parameter Register */
#define DWC3_DGCMDPAR_FORCE_LINKPM_ACCEPT (1 << 0)
#define DWC3_DGCMDPAR_FIFO_NUM(n)         ((n) << 0)
#define DWC3_DGCMDPAR_RX_FIFO             (0 << 5)
#define DWC3_DGCMDPAR_TX_FIFO             (1 << 5)
#define DWC3_DGCMDPAR_LOOPBACK_DIS        (0 << 0)
#define DWC3_DGCMDPAR_LOOPBACK_ENA        (1 << 0)

/* Device Endpoint Command Register */
#define DWC3_DEPCMD_PARAM_SHIFT    16
#define DWC3_DEPCMD_PARAM(x)       ((x) << DWC3_DEPCMD_PARAM_SHIFT)
#define DWC3_DEPCMD_GET_RSC_IDX(x) (((x) >> DWC3_DEPCMD_PARAM_SHIFT) & 0x7f)
#define DWC3_DEPCMD_STATUS(x)      (((x) >> 15) & 1)
#define DWC3_DEPCMD_HIPRI_FORCERM  (1 << 11)
#define DWC3_DEPCMD_CMDACT         (1 << 10)
#define DWC3_DEPCMD_CMDIOC         (1 << 8)

#define DWC3_DEPCMD_DEPSTARTCFG    (0x09 << 0)
#define DWC3_DEPCMD_ENDTRANSFER    (0x08 << 0)
#define DWC3_DEPCMD_UPDATETRANSFER (0x07 << 0)
#define DWC3_DEPCMD_STARTTRANSFER  (0x06 << 0)
#define DWC3_DEPCMD_CLEARSTALL     (0x05 << 0)
#define DWC3_DEPCMD_SETSTALL       (0x04 << 0)
/* This applies for core versions 1.90a and earlier */
#define DWC3_DEPCMD_GETSEQNUMBER (0x03 << 0)
/* This applies for core versions 1.94a and later */
#define DWC3_DEPCMD_GETEPSTATE        (0x03 << 0)
#define DWC3_DEPCMD_SETTRANSFRESOURCE (0x02 << 0)
#define DWC3_DEPCMD_SETEPCONFIG       (0x01 << 0)

/* The EP number goes 0..31 so ep0 is always out and ep1 is always in */
#define DWC3_DALEPENA_EP(n) (1 << n)

#define DWC3_DEPCMD_TYPE_CONTROL 0
#define DWC3_DEPCMD_TYPE_ISOC    1
#define DWC3_DEPCMD_TYPE_BULK    2
#define DWC3_DEPCMD_TYPE_INTR    3

#define DWC3_EVENT_PENDING BIT(0)

#define DWC3_EP_FLAG_STALLED (1 << 0)
#define DWC3_EP_FLAG_WEDGED  (1 << 1)

#define DWC3_EP_DIRECTION_TX true
#define DWC3_EP_DIRECTION_RX false

#define DWC3_TRB_NUM  32
#define DWC3_TRB_MASK (DWC3_TRB_NUM - 1)

#define DWC3_EP_ENABLED         (1 << 0)
#define DWC3_EP_STALL           (1 << 1)
#define DWC3_EP_WEDGE           (1 << 2)
#define DWC3_EP_BUSY            (1 << 4)
#define DWC3_EP_PENDING_REQUEST (1 << 5)
#define DWC3_EP_MISSED_ISOC     (1 << 6)

/* This last one is specific to EP0 */
#define DWC3_EP0_DIR_IN (1 << 31)

enum dwc3_link_state {
    /* In SuperSpeed */
    DWC3_LINK_STATE_U0 = 0x00, /* in HS, means ON */
    DWC3_LINK_STATE_U1 = 0x01,
    DWC3_LINK_STATE_U2 = 0x02, /* in HS, means SLEEP */
    DWC3_LINK_STATE_U3 = 0x03, /* in HS, means SUSPEND */
    DWC3_LINK_STATE_SS_DIS = 0x04,
    DWC3_LINK_STATE_RX_DET = 0x05, /* in HS, means Early Suspend */
    DWC3_LINK_STATE_SS_INACT = 0x06,
    DWC3_LINK_STATE_POLL = 0x07,
    DWC3_LINK_STATE_RECOV = 0x08,
    DWC3_LINK_STATE_HRESET = 0x09,
    DWC3_LINK_STATE_CMPLY = 0x0a,
    DWC3_LINK_STATE_LPBK = 0x0b,
    DWC3_LINK_STATE_RESET = 0x0e,
    DWC3_LINK_STATE_RESUME = 0x0f,
    DWC3_LINK_STATE_MASK = 0x0f,
};

/* TRB Length, PCM and Status */
#define DWC3_TRB_SIZE_MASK      (0x00ffffff)
#define DWC3_TRB_SIZE_LENGTH(n) ((n)&DWC3_TRB_SIZE_MASK)
#define DWC3_TRB_SIZE_PCM1(n)   (((n)&0x03) << 24)
#define DWC3_TRB_SIZE_TRBSTS(n) (((n) & (0x0f << 28)) >> 28)

#define DWC3_TRBSTS_OK            0
#define DWC3_TRBSTS_MISSED_ISOC   1
#define DWC3_TRBSTS_SETUP_PENDING 2
#define DWC3_TRB_STS_XFER_IN_PROG 4

/* TRB Control */
#define DWC3_TRB_CTRL_HWO         (1 << 0)
#define DWC3_TRB_CTRL_LST         (1 << 1)
#define DWC3_TRB_CTRL_CHN         (1 << 2)
#define DWC3_TRB_CTRL_CSP         (1 << 3)
#define DWC3_TRB_CTRL_TRBCTL(n)   (((n)&0x3f) << 4)
#define DWC3_TRB_CTRL_ISP_IMI     (1 << 10)
#define DWC3_TRB_CTRL_IOC         (1 << 11)
#define DWC3_TRB_CTRL_SID_SOFN(n) (((n)&0xffff) << 14)

#define DWC3_TRBCTL_NORMAL            DWC3_TRB_CTRL_TRBCTL(1)
#define DWC3_TRBCTL_CONTROL_SETUP     DWC3_TRB_CTRL_TRBCTL(2)
#define DWC3_TRBCTL_CONTROL_STATUS2   DWC3_TRB_CTRL_TRBCTL(3)
#define DWC3_TRBCTL_CONTROL_STATUS3   DWC3_TRB_CTRL_TRBCTL(4)
#define DWC3_TRBCTL_CONTROL_DATA      DWC3_TRB_CTRL_TRBCTL(5)
#define DWC3_TRBCTL_ISOCHRONOUS_FIRST DWC3_TRB_CTRL_TRBCTL(6)
#define DWC3_TRBCTL_ISOCHRONOUS       DWC3_TRB_CTRL_TRBCTL(7)
#define DWC3_TRBCTL_LINK_TRB          DWC3_TRB_CTRL_TRBCTL(8)

/**
 * struct dwc3_trb - transfer request block (hw format)
 * @bpl: DW0-3
 * @bph: DW4-7
 * @size: DW8-B
 * @trl: DWC-F
 */

/* HWPARAMS0 */
#define DWC3_MODE(n) ((n)&0x7)

#define DWC3_MODE_DEVICE 0
#define DWC3_MODE_HOST   1
#define DWC3_MODE_DRD    2
#define DWC3_MODE_HUB    3

#define DWC3_MDWIDTH(n) (((n)&0xff00) >> 8)

/* HWPARAMS1 */
#define DWC3_NUM_INT(n) (((n) & (0x3f << 15)) >> 15)

/* HWPARAMS3 */
#define DWC3_NUM_IN_EPS_MASK (0x1f << 18)
#define DWC3_NUM_EPS_MASK    (0x3f << 12)
#define DWC3_NUM_EPS(p)      (((p)->hwparams3 & (DWC3_NUM_EPS_MASK)) >> 12)
#define DWC3_NUM_IN_EPS(p)   (((p)->hwparams3 & (DWC3_NUM_IN_EPS_MASK)) >> 18)

/* HWPARAMS7 */
#define DWC3_RAM1_DEPTH(n) ((n)&0xffff)

#define DWC3_REVISION_173A 0x5533173a
#define DWC3_REVISION_175A 0x5533175a
#define DWC3_REVISION_180A 0x5533180a
#define DWC3_REVISION_183A 0x5533183a
#define DWC3_REVISION_185A 0x5533185a
#define DWC3_REVISION_187A 0x5533187a
#define DWC3_REVISION_188A 0x5533188a
#define DWC3_REVISION_190A 0x5533190a
#define DWC3_REVISION_194A 0x5533194a
#define DWC3_REVISION_200A 0x5533200a
#define DWC3_REVISION_202A 0x5533202a
#define DWC3_REVISION_210A 0x5533210a
#define DWC3_REVISION_220A 0x5533220a
#define DWC3_REVISION_230A 0x5533230a
#define DWC3_REVISION_240A 0x5533240a
#define DWC3_REVISION_250A 0x5533250a

/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */

#define DWC3_DEPEVT_XFERCOMPLETE   0x01
#define DWC3_DEPEVT_XFERINPROGRESS 0x02
#define DWC3_DEPEVT_XFERNOTREADY   0x03
#define DWC3_DEPEVT_RXTXFIFOEVT    0x04
#define DWC3_DEPEVT_STREAMEVT      0x06
#define DWC3_DEPEVT_EPCMDCMPLT     0x07

#define DWC3_DEPCFG_EP_TYPE(n)         (((n)&0x3) << 1)
#define DWC3_DEPCFG_EP_NUMBER(n)       (((n)&0x1f) << 25)
#define DWC3_DEPCFG_FIFO_NUMBER(n)     (((n)&0xf) << 17)
#define DWC3_DEPCFG_MAX_PACKET_SIZE(n) (((n)&0x7ff) << 3)

#define DWC3_DEPCFG_INT_NUM(n)          (((n)&0x1f) << 0)
#define DWC3_DEPCFG_XFER_COMPLETE_EN    BIT(8)
#define DWC3_DEPCFG_XFER_IN_PROGRESS_EN BIT(9)
#define DWC3_DEPCFG_XFER_NOT_READY_EN   BIT(10)
#define DWC3_DEPCFG_FIFO_ERROR_EN       BIT(11)
#define DWC3_DEPCFG_STREAM_EVENT_EN     BIT(13)
#define DWC3_DEPCFG_BINTERVAL_M1(n)     (((n)&0xff) << 16)
#define DWC3_DEPCFG_STREAM_CAPABLE      BIT(24)
#define DWC3_DEPCFG_EP_NUMBER(n)        (((n)&0x1f) << 25)
#define DWC3_DEPCFG_BULK_BASED          BIT(30)
#define DWC3_DEPCFG_FIFO_BASED          BIT(31)

#endif /* __DRIVERS_USB_DWC3_CORE_H */
/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */
/*
 *
 * Copyright (C) 2017-21 Corellium LLC
 * All rights reserved.
 *
 */

#include "libc.h"
#include "dtree.h"
#include "adtree.h"
#include "tunable.h"

#define BOOT_LINE_LENGTH        256
struct iphone_boot_args {
    uint16_t revision;
    uint16_t version;
    uint64_t virt_base;
    uint64_t phys_base;
    uint64_t mem_size;
    uint64_t top_of_kernel;
    struct {
        uint64_t phys, display, stride;
        uint64_t width, height, depth;
    } video;
    uint32_t machine_type;
    uint64_t dtree_virt; 
    uint32_t dtree_size;
    char cmdline[BOOT_LINE_LENGTH];
};

void setarena(uint64_t base, uint64_t size);

#define DISP0_SURF0_PIXFMT      0x230850030

static void configure_x8r8g8b8(void)
{
    *(volatile uint32_t *)DISP0_SURF0_PIXFMT = 0x5000; /* pixfmt: x8r8g8b8. if you try other modes they are pretty ugly */
}

static const struct tunable_fuse_map m1_acio_fuse_map[] = {
    /* src_addr, dst_offs, src_[lsb,width], dst_[lsb,width] */
    { 0x23d2bc438, 0x2a38, 19, 6,  0, 7 },
    { 0x23d2bc438, 0x2a38, 25, 6, 17, 7 },
    { 0x23d2bc438, 0x2aa4, 31, 1, 17, 2 },
    { 0x23d2bc438, 0x0a04, 14, 5,  2, 5 },
    { 0x23d2bc43c, 0x2aa4,  0, 1, 17, 2 },
    { 0x23d2bc43c, 0x2a20,  1, 3, 14, 3 },
    { 0x23d2bc438, 0x222c,  7, 2,  9, 2 },
    { 0x23d2bc438, 0x222c,  4, 3, 12, 3 },
    { 0x23d2bc438, 0x22a4, 12, 2, 17, 2 },
    { 0x23d2bc438, 0x2220,  9, 3, 14, 3 },
    { 0x23d2bc438, 0x0a04, 14, 5,  2, 5 },
    { 0 } };

static void prepare_atc_tunables(dtree *apple_dt, dtree *linux_dt, int atc)
{
    char src_phy[24], src_usb[24], src_cio[16], dst_phy[16], dst_usb[16], dst_cio[12];
    char src_pxc[24], src_pxb[32], dst_pxc[12];
    uint64_t base = atc ? 0x500000000ul : 0x380000000ul;

    strcpy(src_phy, "/arm-io/atc-phy0"); src_phy[15] += atc;
    strcpy(src_usb, "/arm-io/usb-drd0"); src_usb[15] += atc;
    strcpy(src_cio, "/arm-io/acio0");    src_cio[12] += atc;
    strcpy(src_pxc, "/arm-io/apciec0");  src_pxc[14] += atc;
    strcpy(src_pxb, "/arm-io/apciec0/pcic0-bridge");  src_pxb[14] += atc; src_pxb[20] += atc;
    strcpy(dst_phy, "/soc/atcphy0");     dst_phy[11] += atc;
    strcpy(dst_usb, "/soc/usb_drd0");    dst_usb[12] += atc;
    strcpy(dst_cio, "/soc/acio0");       dst_cio[9]  += atc;
    strcpy(dst_pxc, "/soc/pciec0");      dst_pxc[10] += atc;

    prepare_tunable(apple_dt, src_phy, "tunable_ATC0AXI2AF",             linux_dt, dst_usb, "tunable-ATC0AXI2AF",            TUNABLE_FANCY, base);
    prepare_tunable(apple_dt, src_usb, "tunable",                        linux_dt, dst_usb, "tunable",                       TUNABLE_LEGACY, 0);

    prepare_tunable(apple_dt, src_phy, "tunable_ATC0AXI2AF",             linux_dt, dst_phy, "tunable-ATC0AXI2AF",            TUNABLE_FANCY, base);
    prepare_tunable(apple_dt, src_phy, "tunable_ATC_FABRIC",             linux_dt, dst_phy, "tunable-ATC_FABRIC",            TUNABLE_FANCY, base + 0x3045000);
    prepare_tunable(apple_dt, src_phy, "tunable_AUS_CMN_SHM",            linux_dt, dst_phy, "tunable-AUS_CMN_SHM",           TUNABLE_FANCY, base + 0x3000a00);
    prepare_tunable(apple_dt, src_phy, "tunable_AUS_CMN_TOP",            linux_dt, dst_phy, "tunable-AUS_CMN_TOP",           TUNABLE_FANCY, base + 0x3000800);
    prepare_tunable(apple_dt, src_phy, "tunable_AUSPLL_CORE",            linux_dt, dst_phy, "tunable-AUSPLL_CORE",           TUNABLE_FANCY, base + 0x3002200);
    prepare_tunable(apple_dt, src_phy, "tunable_AUSPLL_TOP",             linux_dt, dst_phy, "tunable-AUSPLL_TOP",            TUNABLE_FANCY, base + 0x3002000);
    prepare_tunable(apple_dt, src_phy, "tunable_CIO3PLL_CORE",           linux_dt, dst_phy, "tunable-CIO3PLL_CORE",          TUNABLE_FANCY, base + 0x3002a00);
    prepare_tunable(apple_dt, src_phy, "tunable_CIO3PLL_TOP",            linux_dt, dst_phy, "tunable-CIO3PLL_TOP",           TUNABLE_FANCY, base + 0x3002800);
    prepare_tunable(apple_dt, src_phy, "tunable_CIO_LN0_AUSPMA_RX_EQ",   linux_dt, dst_phy, "tunable-CIO_LN0_AUSPMA_RX_EQ",  TUNABLE_FANCY, base + 0x300a000);
    prepare_tunable(apple_dt, src_phy, "tunable_USB_LN0_AUSPMA_RX_EQ",   linux_dt, dst_phy, "tunable-USB_LN0_AUSPMA_RX_EQ",  TUNABLE_FANCY, base + 0x300a000);
    prepare_tunable(apple_dt, src_phy, "tunable_CIO_LN0_AUSPMA_RX_SHM",  linux_dt, dst_phy, "tunable-CIO_LN0_AUSPMA_RX_SHM", TUNABLE_FANCY, base + 0x300b000);
    prepare_tunable(apple_dt, src_phy, "tunable_USB_LN0_AUSPMA_RX_SHM",  linux_dt, dst_phy, "tunable-USB_LN0_AUSPMA_RX_SHM", TUNABLE_FANCY, base + 0x300b000);
    prepare_tunable(apple_dt, src_phy, "tunable_CIO_LN0_AUSPMA_RX_TOP",  linux_dt, dst_phy, "tunable-CIO_LN0_AUSPMA_RX_TOP", TUNABLE_FANCY, base + 0x3009000);
    prepare_tunable(apple_dt, src_phy, "tunable_USB_LN0_AUSPMA_RX_TOP",  linux_dt, dst_phy, "tunable-USB_LN0_AUSPMA_RX_TOP", TUNABLE_FANCY, base + 0x3009000);
    prepare_tunable(apple_dt, src_phy, "tunable_CIO_LN0_AUSPMA_TX_TOP",  linux_dt, dst_phy, "tunable-CIO_LN0_AUSPMA_TX_TOP", TUNABLE_FANCY, base + 0x300c000);
    prepare_tunable(apple_dt, src_phy, "tunable_DP_LN0_AUSPMA_TX_TOP",   linux_dt, dst_phy, "tunable-DP_LN0_AUSPMA_TX_TOP",  TUNABLE_FANCY, base + 0x300c000);
    prepare_tunable(apple_dt, src_phy, "tunable_USB_LN0_AUSPMA_TX_TOP",  linux_dt, dst_phy, "tunable-USB_LN0_AUSPMA_TX_TOP", TUNABLE_FANCY, base + 0x300c000);
    prepare_tunable(apple_dt, src_phy, "tunable_CIO_LN1_AUSPMA_RX_EQ",   linux_dt, dst_phy, "tunable-CIO_LN1_AUSPMA_RX_EQ",  TUNABLE_FANCY, base + 0x3011000);
    prepare_tunable(apple_dt, src_phy, "tunable_USB_LN1_AUSPMA_RX_EQ",   linux_dt, dst_phy, "tunable-USB_LN1_AUSPMA_RX_EQ",  TUNABLE_FANCY, base + 0x3011000);
    prepare_tunable(apple_dt, src_phy, "tunable_CIO_LN1_AUSPMA_RX_SHM",  linux_dt, dst_phy, "tunable-CIO_LN1_AUSPMA_RX_SHM", TUNABLE_FANCY, base + 0x3012000);
    prepare_tunable(apple_dt, src_phy, "tunable_USB_LN1_AUSPMA_RX_SHM",  linux_dt, dst_phy, "tunable-USB_LN1_AUSPMA_RX_SHM", TUNABLE_FANCY, base + 0x3012000);
    prepare_tunable(apple_dt, src_phy, "tunable_CIO_LN1_AUSPMA_RX_TOP",  linux_dt, dst_phy, "tunable-CIO_LN1_AUSPMA_RX_TOP", TUNABLE_FANCY, base + 0x3010000);
    prepare_tunable(apple_dt, src_phy, "tunable_USB_LN1_AUSPMA_RX_TOP",  linux_dt, dst_phy, "tunable-USB_LN1_AUSPMA_RX_TOP", TUNABLE_FANCY, base + 0x3010000);
    prepare_tunable(apple_dt, src_phy, "tunable_CIO_LN1_AUSPMA_TX_TOP",  linux_dt, dst_phy, "tunable-CIO_LN1_AUSPMA_TX_TOP", TUNABLE_FANCY, base + 0x3013000);
    prepare_tunable(apple_dt, src_phy, "tunable_DP_LN1_AUSPMA_TX_TOP",   linux_dt, dst_phy, "tunable-DP_LN1_AUSPMA_TX_TOP",  TUNABLE_FANCY, base + 0x3013000);
    prepare_tunable(apple_dt, src_phy, "tunable_USB_LN1_AUSPMA_TX_TOP",  linux_dt, dst_phy, "tunable-USB_LN1_AUSPMA_TX_TOP", TUNABLE_FANCY, base + 0x3013000);
    prepare_tunable(apple_dt, src_phy, "tunable_USB_ACIOPHY_TOP",        linux_dt, dst_phy, "tunable-USB_ACIOPHY_TOP",       TUNABLE_FANCY, base + 0x3000000);

    prepare_fuse_tunable(linux_dt, dst_phy, "tunable-fuse", m1_acio_fuse_map, base + 0x3000000);

    prepare_tunable(apple_dt, src_cio, "fw_int_ctl_management_tunables", linux_dt, dst_cio, "tunable-fw_int_ctl_management", TUNABLE_PCIE, 0 | 0x04000);
    prepare_tunable(apple_dt, src_cio, "hbw_fabric_tunables",            linux_dt, dst_cio, "tunable-hbw_fabric",            TUNABLE_PCIE, 3);
    prepare_tunable(apple_dt, src_cio, "hi_dn_merge_fabric_tunables",    linux_dt, dst_cio, "tunable-hi_dn_merge_fabric",    TUNABLE_PCIE, 0 | 0xfc000);
    prepare_tunable(apple_dt, src_cio, "hi_up_merge_fabric_tunables",    linux_dt, dst_cio, "tunable-hi_up_merge_fabric",    TUNABLE_PCIE, 0 | 0xf8000);
    prepare_tunable(apple_dt, src_cio, "hi_up_tx_desc_fabric_tunables",  linux_dt, dst_cio, "tunable-hi_up_tx_desc_fabric",  TUNABLE_PCIE, 0 | 0xf0000);
    prepare_tunable(apple_dt, src_cio, "hi_up_tx_data_fabric_tunables",  linux_dt, dst_cio, "tunable-hi_up_tx_data_fabric",  TUNABLE_PCIE, 0 | 0xec000);
    prepare_tunable(apple_dt, src_cio, "hi_up_rx_desc_fabric_tunables",  linux_dt, dst_cio, "tunable-hi_up_rx_desc_fabric",  TUNABLE_PCIE, 0 | 0xe8000);
    prepare_tunable(apple_dt, src_cio, "hi_up_wr_fabric_tunables",       linux_dt, dst_cio, "tunable-hi_up_wr_fabric",       TUNABLE_PCIE, 0 | 0xf4000);
    prepare_tunable(apple_dt, src_cio, "lbw_fabric_tunables",            linux_dt, dst_cio, "tunable-lbw_fabric",            TUNABLE_PCIE, 4);
    prepare_tunable(apple_dt, src_cio, "pcie_adapter_regs_tunables",     linux_dt, dst_cio, "tunable-pcie_adapter_regs",     TUNABLE_PCIE, 5);
    prepare_tunable(apple_dt, src_cio, "top_tunables",                   linux_dt, dst_cio, "tunable-top",                   TUNABLE_PCIE, 2);

    prepare_tunable(apple_dt, src_cio, "thunderbolt-drom",               linux_dt, dst_cio, "thunderbolt-drom",              TUNABLE_PLAIN, PLAIN_BYTE);

    prepare_tunable(apple_dt, src_pxc, "atc-apcie-debug-tunables",       linux_dt, dst_pxc, "tunable-debug",                 TUNABLE_PCIE, 6);
    prepare_tunable(apple_dt, src_pxc, "atc-apcie-fabric-tunables",      linux_dt, dst_pxc, "tunable-fabric",                TUNABLE_PCIE, 4);
    prepare_tunable(apple_dt, src_pxc, "atc-apcie-oe-fabric-tunables",   linux_dt, dst_pxc, "tunable-oe-fabric",             TUNABLE_PCIE, 5);
    prepare_tunable(apple_dt, src_pxc, "atc-apcie-rc-tunables",          linux_dt, dst_pxc, "tunable-rc",                    TUNABLE_PCIE, 0);
    prepare_tunable(apple_dt, src_pxb, "apcie-config-tunables",          linux_dt, dst_pxc, "tunable-port0-config",          TUNABLE_PCIE_PARENT, 3);
}

static const struct tunable_fuse_map m1_pcie_fuse_map[] = {
    /* src_addr, dst_offs, src_[lsb,width], dst_[lsb,width] */
    { 0x23d2bc084, 0x6238,  4, 6,  0, 7 },
    { 0x23d2bc084, 0x6220, 10, 3, 14, 3 },
    { 0x23d2bc084, 0x62a4, 13, 2, 17, 2 },
    { 0x23d2bc418, 0x522c, 27, 2,  9, 2 },
    { 0x23d2bc418, 0x522c, 13, 3, 12, 3 },
    { 0x23d2bc418, 0x5220, 18, 3, 14, 3 },
    { 0x23d2bc418, 0x52a4, 21, 2, 17, 2 },
    { 0x23d2bc418, 0x522c, 23, 5, 16, 5 },
    { 0x23d2bc418, 0x5278, 23, 3, 20, 3 },
    { 0x23d2bc418, 0x5018, 31, 1,  2, 1 },
    { 0x23d2bc41c, 0x1204,  0, 5,  2, 5 },
    { 0 } };

void enable_all_clocks(void)
{
    *(volatile uint32_t *)(0x23b700108) = 0xff;
    *(volatile uint32_t *)(0x23b700110) = 0xff;
    *(volatile uint32_t *)(0x23b700118) = 0xff;
    *(volatile uint32_t *)(0x23b700120) = 0xff;
    *(volatile uint32_t *)(0x23b700128) = 0xff;
    *(volatile uint32_t *)(0x23b700130) = 0xff;
    *(volatile uint32_t *)(0x23b700138) = 0xff;
    *(volatile uint32_t *)(0x23b700140) = 0xff;
    *(volatile uint32_t *)(0x23b700148) = 0xff;
    *(volatile uint32_t *)(0x23b700150) = 0xff;
    *(volatile uint32_t *)(0x23b700158) = 0xff;
    *(volatile uint32_t *)(0x23b700160) = 0xff;
    *(volatile uint32_t *)(0x23b700168) = 0xff;
    *(volatile uint32_t *)(0x23b700170) = 0xff;
    *(volatile uint32_t *)(0x23b700178) = 0xff;
    *(volatile uint32_t *)(0x23b700180) = 0xff;
    *(volatile uint32_t *)(0x23b700188) = 0xff;
    *(volatile uint32_t *)(0x23b700190) = 0xff;
    *(volatile uint32_t *)(0x23b700198) = 0xff;
    *(volatile uint32_t *)(0x23b7001a0) = 0xff;
    *(volatile uint32_t *)(0x23b7001a8) = 0xff;
    *(volatile uint32_t *)(0x23b7001b0) = 0xff;
    *(volatile uint32_t *)(0x23b7001b8) = 0xff;
    *(volatile uint32_t *)(0x23b7001c0) = 0xff;
    *(volatile uint32_t *)(0x23b7001c8) = 0xff;
    *(volatile uint32_t *)(0x23b7001d0) = 0xff;
    *(volatile uint32_t *)(0x23b7001d8) = 0xff;
    *(volatile uint32_t *)(0x23b7001e0) = 0xff;
    *(volatile uint32_t *)(0x23b7001e8) = 0xff;
    *(volatile uint32_t *)(0x23b7001f0) = 0xff;
    *(volatile uint32_t *)(0x23b7001f8) = 0xff;
    *(volatile uint32_t *)(0x23b700200) = 0xff;
    *(volatile uint32_t *)(0x23b700208) = 0xff;
    *(volatile uint32_t *)(0x23b700210) = 0xff;
    *(volatile uint32_t *)(0x23b700218) = 0xff;
    *(volatile uint32_t *)(0x23b700220) = 0xff;
    *(volatile uint32_t *)(0x23b700228) = 0xff;
    *(volatile uint32_t *)(0x23b700230) = 0xff;
    *(volatile uint32_t *)(0x23b700238) = 0xff;
    *(volatile uint32_t *)(0x23b700240) = 0xff;
    *(volatile uint32_t *)(0x23b700248) = 0xff;
    *(volatile uint32_t *)(0x23b700250) = 0xff;
    *(volatile uint32_t *)(0x23b700258) = 0xff;
    *(volatile uint32_t *)(0x23b700260) = 0xff;
    *(volatile uint32_t *)(0x23b700268) = 0xff;
    *(volatile uint32_t *)(0x23b700270) = 0xff;
    *(volatile uint32_t *)(0x23b700278) = 0xff;
    *(volatile uint32_t *)(0x23b700280) = 0xff;
    *(volatile uint32_t *)(0x23b700288) = 0xff;
    *(volatile uint32_t *)(0x23b700290) = 0xff;
    *(volatile uint32_t *)(0x23b700298) = 0xff;
    *(volatile uint32_t *)(0x23b7002a0) = 0xff;
    *(volatile uint32_t *)(0x23b7002a8) = 0xff;
    *(volatile uint32_t *)(0x23b7002b0) = 0xff;
    *(volatile uint32_t *)(0x23b7002b8) = 0xff;
    *(volatile uint32_t *)(0x23b7002c0) = 0xff;
    *(volatile uint32_t *)(0x23b7002c8) = 0xff;
    *(volatile uint32_t *)(0x23b7002d0) = 0xff;
    *(volatile uint32_t *)(0x23b7002d8) = 0xff;
    *(volatile uint32_t *)(0x23b7002e0) = 0xff;
    *(volatile uint32_t *)(0x23b7002e8) = 0xff;
    *(volatile uint32_t *)(0x23b7002f0) = 0xff;
    *(volatile uint32_t *)(0x23b7002f8) = 0xff;
    *(volatile uint32_t *)(0x23b700300) = 0xff;
    *(volatile uint32_t *)(0x23b700308) = 0xff;
    *(volatile uint32_t *)(0x23b700310) = 0xff;
    *(volatile uint32_t *)(0x23b700318) = 0xff;
    *(volatile uint32_t *)(0x23b700320) = 0xff;
    *(volatile uint32_t *)(0x23b700328) = 0xff;
    *(volatile uint32_t *)(0x23b700330) = 0xff;
    *(volatile uint32_t *)(0x23b700338) = 0xff;
    *(volatile uint32_t *)(0x23b700340) = 0xff;
    *(volatile uint32_t *)(0x23b700348) = 0xff;
    *(volatile uint32_t *)(0x23b700350) = 0xff;
    *(volatile uint32_t *)(0x23b700358) = 0xff;
    *(volatile uint32_t *)(0x23b700360) = 0xff;
    *(volatile uint32_t *)(0x23b700368) = 0xff;
    *(volatile uint32_t *)(0x23b700370) = 0xff;
    *(volatile uint32_t *)(0x23b700378) = 0xff;
    *(volatile uint32_t *)(0x23b700380) = 0xff;
    *(volatile uint32_t *)(0x23b700388) = 0xff;
    *(volatile uint32_t *)(0x23b700390) = 0xff;
    *(volatile uint32_t *)(0x23b700398) = 0xff;
    *(volatile uint32_t *)(0x23b7003a0) = 0xff;
    *(volatile uint32_t *)(0x23b7003a8) = 0xff;
    *(volatile uint32_t *)(0x23b7003b0) = 0xff;
    *(volatile uint32_t *)(0x23b7003b8) = 0xff;
    *(volatile uint32_t *)(0x23b7003c0) = 0xff;
    *(volatile uint32_t *)(0x23b7003c8) = 0xff;
    *(volatile uint32_t *)(0x23b7003d0) = 0xff;
    *(volatile uint32_t *)(0x23b7003d8) = 0xff;
    *(volatile uint32_t *)(0x23b7003e0) = 0xff;
    *(volatile uint32_t *)(0x23b7003e8) = 0xff;
    *(volatile uint32_t *)(0x23b7003f0) = 0xff;
    *(volatile uint32_t *)(0x23b7003f8) = 0xff;
    *(volatile uint32_t *)(0x23b700400) = 0xff;
    *(volatile uint32_t *)(0x23b700408) = 0xff;
    *(volatile uint32_t *)(0x23b700410) = 0xff;
    *(volatile uint32_t *)(0x23b700418) = 0xff;
    *(volatile uint32_t *)(0x23b700420) = 0xff;
    *(volatile uint32_t *)(0x23b700428) = 0xff;
    *(volatile uint32_t *)(0x23b700430) = 0xff;
    *(volatile uint32_t *)(0x23b700438) = 0xff;
    *(volatile uint32_t *)(0x23b700440) = 0xff;
    *(volatile uint32_t *)(0x23b700448) = 0xff;
    *(volatile uint32_t *)(0x23b700450) = 0xff;
    *(volatile uint32_t *)(0x23b700458) = 0xff;
    *(volatile uint32_t *)(0x23b700460) = 0xff;
    *(volatile uint32_t *)(0x23b700468) = 0xff;
    *(volatile uint32_t *)(0x23b700470) = 0xff;
    *(volatile uint32_t *)(0x23b700478) = 0xff;
    *(volatile uint32_t *)(0x23b700c08) = 0xff;
    *(volatile uint32_t *)(0x23b700c10) = 0xff;
    *(volatile uint32_t *)(0x23d280060) = 0xff;
    *(volatile uint32_t *)(0x23d280068) = 0xff;
    *(volatile uint32_t *)(0x23d280070) = 0xff;
    *(volatile uint32_t *)(0x23d280078) = 0xff;
    *(volatile uint32_t *)(0x23d280080) = 0xff;
    *(volatile uint32_t *)(0x23d280088) = 0xff;
    *(volatile uint32_t *)(0x23d280090) = 0xff;
    *(volatile uint32_t *)(0x23d280098) = 0xff;
    *(volatile uint32_t *)(0x23d2800a0) = 0xff;
    *(volatile uint32_t *)(0x23d2800a8) = 0xff;
    *(volatile uint32_t *)(0x23d2800b0) = 0xff;
    *(volatile uint32_t *)(0x23d2800b8) = 0xff;
    *(volatile uint32_t *)(0x23d2800c0) = 0xff;
    *(volatile uint32_t *)(0x23d2800c8) = 0xff;
}

void bring_up_phys(void)
{
#if 0
    *(volatile uint32_t *)0x382a90008 = 0x1c1000f;
    *(volatile uint32_t *)0x382a90004 = 3;
    *(volatile uint32_t *)0x382a90004 = 0;
    *(volatile uint32_t *)0x382a9001c = 0x8c0813;
    *(volatile uint32_t *)0x382a90000 = 2;

    *(volatile uint32_t *)0x382a8400c = 2;
    *(volatile uint32_t *)0x382a8400c = 0x22;
    *(volatile uint32_t *)0x382a8401c = 0x21;
    *(volatile uint32_t *)0x382a84020 = 0x9332;
#endif

    *(volatile uint32_t *)0x502a90008 = 0x1c1000f;
    *(volatile uint32_t *)0x502a90004 = 3;
    *(volatile uint32_t *)0x502a90004 = 0;
    *(volatile uint32_t *)0x502a9001c = 0x8c0813;
    *(volatile uint32_t *)0x502a90000 = 2;

    *(volatile uint32_t *)0x502a8400c = 2;
    *(volatile uint32_t *)0x502a8400c = 0x22;
    *(volatile uint32_t *)0x502a8401c = 0x21;
    *(volatile uint32_t *)0x502a84020 = 0x9332;
}

void soft_reset_dwc3s(void)
{
    void *addrs[] = { (void *)0x382280000, (void *)0x502280000 };
    for (int i = 1; i < 2; i++) {
	void *addr = addrs[i];
#define set32(a,b) (*(volatile uint32_t *)(a)) |= (b)
#define clear32(a,b) (*(volatile uint32_t *)(a)) &= ~(b)
	set32(addr + DWC3_DCTL, DWC3_DCTL_CSFTRST);
	udelay(10*1000);
	/* soft reset the core and phy */
	set32(addr + DWC3_GCTL, DWC3_GCTL_CORESOFTRESET);
	set32(addr + DWC3_GUSB3PIPECTL(0), DWC3_GUSB3PIPECTL_PHYSOFTRST);
	set32(addr + DWC3_GUSB2PHYCFG(0), DWC3_GUSB2PHYCFG_PHYSOFTRST);
	udelay(100*1000);
	clear32(addr + DWC3_GUSB3PIPECTL(0), DWC3_GUSB3PIPECTL_PHYSOFTRST);
	clear32(addr + DWC3_GUSB2PHYCFG(0), DWC3_GUSB2PHYCFG_PHYSOFTRST);
	udelay(100*1000);
	clear32(addr + DWC3_GCTL, DWC3_GCTL_CORESOFTRESET);
	udelay(100*1000);

	/* disable unused features */
	clear32(addr + DWC3_GCTL, DWC3_GCTL_SCALEDOWN_MASK | DWC3_GCTL_DISSCRAMBLE);
    }
}

void loader_main(void *linux_dtb, struct iphone_boot_args *bootargs, uint64_t smpentry, uint64_t rvbar)
{
    dtree *linux_dt, *apple_dt;
    dt_node *node;
    dt_prop *prop;
    uint64_t memsize, base, size;
    unsigned i;
    char *model;

    warning_count = 0;
    memsize = (bootargs->mem_size + 0x3ffffffful) & ~0x3ffffffful;

    setarena(0x880000000ul, 0x400000);
    setvideo((void *)bootargs->video.phys, bootargs->video.width, bootargs->video.height, bootargs->video.stride);

    printf("Starting Linux loader stub.\n");

    printf("Enabling all clocks.\n");
    enable_all_clocks();
    udelay(1000);
    enable_all_clocks();
    udelay(1000);
    enable_all_clocks();
    printf("Bringing up PHYs\n");
    udelay(1000);
    bring_up_phys();
    udelay(1000);
    soft_reset_dwc3s();
    udelay(1000);

    linux_dt = dt_parse_dtb(linux_dtb, 0x20000);
    if(!linux_dt) {
        printf("Failed parsing Linux device-tree.\n");
        return;
    }

    apple_dt = adt_parse((void *)(bootargs->dtree_virt - bootargs->virt_base + bootargs->phys_base), bootargs->dtree_size);
    if(!apple_dt) {
        printf("Failed parsing Apple device-tree.\n");
        warning_count ++;
    }

    node = dt_find_node(linux_dt, "/framebuffer");
    if(node) {
        prop = dt_find_prop(linux_dt, node, "reg");
        if(prop) {
            dt_put64be(prop->buf, bootargs->video.phys);
            dt_put64be(prop->buf + 8, (bootargs->video.height * bootargs->video.stride * 4 + 0x3FFFul) & ~0x3FFFul);
        }

        prop = dt_find_prop(linux_dt, node, "width");
        if(prop)
            dt_put32be(prop->buf, bootargs->video.width);

        prop = dt_find_prop(linux_dt, node, "height");
        if(prop)
            dt_put32be(prop->buf, bootargs->video.height);

        prop = dt_find_prop(linux_dt, node, "stride");
        if(prop)
            dt_put32be(prop->buf, bootargs->video.stride);

        dt_set_prop(linux_dt, node, "format", "x8r8g8b8", 9);
    }

    node = dt_find_node(linux_dt, "/memory");
    if(node) {
        prop = dt_find_prop(linux_dt, node, "reg");
        if(prop)
            dt_put64be(prop->buf + 8, memsize);
    }

    for(i=0; ; i++) {
        node = dt_find_node_idx(linux_dt, NULL, "/reserved-memory/fw_area", i);
        if(!node)
            break;

        prop = dt_find_prop(linux_dt, node, "reg");
        if(prop) {
            base = dt_get64be(prop->buf);
            if(base >= 0x900000000ul) { /* high range */
                base = bootargs->phys_base + bootargs->mem_size;
                size = (0x800000000ul + memsize) - base;
            } else { /* low range */
                base = 0x800000000ul;
                size = bootargs->phys_base - base;
            }
            dt_put64be(prop->buf, base);
            dt_put64be(prop->buf + 8, size);
        }
    }

    node = dt_find_node(linux_dt, "/reserved-memory/smpentry");
    if(node) {
        prop = dt_find_prop(linux_dt, node, "reg");
        if(prop)
            dt_put64be(prop->buf, smpentry);
    }

    node = dt_find_node(linux_dt, "/reserved-memory/adt");
    if(node) {
        prop = dt_find_prop(linux_dt, node, "reg");
        if(prop) {
	    dt_put64be(prop->buf, bootargs->dtree_virt - bootargs->virt_base + bootargs->phys_base);
	    dt_put64be(prop->buf + 8, bootargs->dtree_size);
	}
    }

    node = dt_find_node(linux_dt, "/reserved-memory/bootargs");
    if(node) {
        prop = dt_find_prop(linux_dt, node, "reg");
        if(prop) {
	  dt_put64be(prop->buf, bootargs);
	  dt_put64be(prop->buf + 8, 16384);
	}
    }
    node = dt_find_node(linux_dt, "/adt");
    if(node) {
        prop = dt_set_prop(linux_dt, node, "contents", (void *)bootargs->dtree_virt - bootargs->virt_base + bootargs->phys_base, bootargs->dtree_size);
    }

#if 0
    node = dt_find_node(linux_dt, "/soc/applestart");
    if(node) {
        prop = dt_find_prop(linux_dt, node, "reg");
        if(prop)
            for(i=0; i<prop->size/48; i++)
                dt_put64be(prop->buf + 48 * i + 16, rvbar + 8 * i);
    }
#endif

    model = NULL;
    if(apple_dt) {
        node = dt_find_node(apple_dt, "/");
        if(node) {
            prop = dt_find_prop(apple_dt, node, "target-type");
            if(prop)
                model = prop->buf;
        }
    }
    if(!model) {
        model = "unknown";
        warning_count ++;
    }

    printf("Running on '%s'...\n", model);

    prepare_atc_tunables(apple_dt, linux_dt, 0);
    prepare_atc_tunables(apple_dt, linux_dt, 1);

    prepare_tunable(apple_dt, "/arm-io/apcie",             "apcie-axi2af-tunables",        linux_dt, "/soc/pcie", "tunable-axi2af",            TUNABLE_PCIE, 4);
    prepare_tunable(apple_dt, "/arm-io/apcie",             "apcie-common-tunables",        linux_dt, "/soc/pcie", "tunable-common",            TUNABLE_PCIE, 1);
    prepare_tunable(apple_dt, "/arm-io/apcie",             "apcie-phy-ip-auspma-tunables", linux_dt, "/soc/pcie", "tunable-phy-ip-auspma",     TUNABLE_PCIE, 3);
    prepare_tunable(apple_dt, "/arm-io/apcie",             "apcie-phy-ip-pll-tunables",    linux_dt, "/soc/pcie", "tunable-phy-ip-pll",        TUNABLE_PCIE, 3);
    prepare_tunable(apple_dt, "/arm-io/apcie",             "apcie-phy-tunables",           linux_dt, "/soc/pcie", "tunable-phy",               TUNABLE_PCIE, 2);
    prepare_tunable(apple_dt, "/arm-io/apcie/pci-bridge0", "apcie-config-tunables",        linux_dt, "/soc/pcie", "tunable-port0-config",      TUNABLE_PCIE_PARENT, 6);
    prepare_tunable(apple_dt, "/arm-io/apcie/pci-bridge0", "pcie-rc-gen3-shadow-tunables", linux_dt, "/soc/pcie", "tunable-port0-gen3-shadow", TUNABLE_PCIE_PARENT, 0);
    prepare_tunable(apple_dt, "/arm-io/apcie/pci-bridge0", "pcie-rc-gen4-shadow-tunables", linux_dt, "/soc/pcie", "tunable-port0-gen4-shadow", TUNABLE_PCIE_PARENT, 0);
    prepare_tunable(apple_dt, "/arm-io/apcie/pci-bridge0", "pcie-rc-tunables",             linux_dt, "/soc/pcie", "tunable-port0",             TUNABLE_PCIE_PARENT, 0);
    if(!strcmp(model, "J274")) {
        /* Mac Mini has xHCI and Ethernet */
        prepare_tunable(apple_dt, "/arm-io/apcie/pci-bridge1", "apcie-config-tunables",        linux_dt, "/soc/pcie", "tunable-port1-config",      TUNABLE_PCIE_PARENT, 10);
        prepare_tunable(apple_dt, "/arm-io/apcie/pci-bridge1", "pcie-rc-gen3-shadow-tunables", linux_dt, "/soc/pcie", "tunable-port1-gen3-shadow", TUNABLE_PCIE_PARENT, 0 | 0x8000);
        prepare_tunable(apple_dt, "/arm-io/apcie/pci-bridge1", "pcie-rc-gen4-shadow-tunables", linux_dt, "/soc/pcie", "tunable-port1-gen4-shadow", TUNABLE_PCIE_PARENT, 0 | 0x8000);
        prepare_tunable(apple_dt, "/arm-io/apcie/pci-bridge1", "pcie-rc-tunables",             linux_dt, "/soc/pcie", "tunable-port1",             TUNABLE_PCIE_PARENT, 0 | 0x8000);
        prepare_tunable(apple_dt, "/arm-io/apcie/pci-bridge2", "apcie-config-tunables",        linux_dt, "/soc/pcie", "tunable-port2-config",      TUNABLE_PCIE_PARENT, 14);
        prepare_tunable(apple_dt, "/arm-io/apcie/pci-bridge2", "pcie-rc-gen3-shadow-tunables", linux_dt, "/soc/pcie", "tunable-port2-gen3-shadow", TUNABLE_PCIE_PARENT, 0 | 0x10000);
        prepare_tunable(apple_dt, "/arm-io/apcie/pci-bridge2", "pcie-rc-gen4-shadow-tunables", linux_dt, "/soc/pcie", "tunable-port2-gen4-shadow", TUNABLE_PCIE_PARENT, 0 | 0x10000);
        prepare_tunable(apple_dt, "/arm-io/apcie/pci-bridge2", "pcie-rc-tunables",             linux_dt, "/soc/pcie", "tunable-port2",             TUNABLE_PCIE_PARENT, 0 | 0x10000);

        prepare_tunable(apple_dt, "/chosen", "mac-address-ethernet0", linux_dt, "/chosen", "hwaddr-eth0", TUNABLE_PLAIN, PLAIN_BYTE);

        /* remove keyboard and touchbar */
        node = dt_find_node(linux_dt, "/soc/spi@235100000");
        if(node)
            dt_delete_node(node);
        node = dt_find_node(linux_dt, "/soc/spi@23510c000");
        if(node)
            dt_delete_node(node);
    } else {
        /* don't waste time bringing these up on Macbooks */
        node = dt_find_node(linux_dt, "/soc/pcie");
        if(node) {
            prop = dt_find_prop(linux_dt, node, "devpwr-on-1");
            if(prop)
                dt_delete_prop(prop);
            prop = dt_find_prop(linux_dt, node, "devpwr-on-2");
            if(prop)
                dt_delete_prop(prop);
        }
    }
    prepare_fuse_tunable(linux_dt, "/soc/pcie", "tunable-fuse", m1_pcie_fuse_map, 0x6800c0000);

    prepare_tunable(apple_dt, "/arm-io/pmgr", "voltage-states1",         linux_dt, "/soc/cpufreq", "tunable-ecpu-states",    TUNABLE_PLAIN, PLAIN_WORD);
    prepare_tunable(apple_dt, "/arm-io/pmgr", "voltage-states5",         linux_dt, "/soc/cpufreq", "tunable-pcpu-states",    TUNABLE_PLAIN, PLAIN_WORD);
    prepare_tunable(apple_dt, "/arm-io/pmgr", "mcx-fast-pcpu-frequency", linux_dt, "/soc/cpufreq", "tunable-pcpu-fast-freq", TUNABLE_PLAIN, PLAIN_WORD);
    prepare_tunable(apple_dt, "/arm-io/mcc",  "dramcfg-data",            linux_dt, "/soc/cpufreq", "tunable-pcpu-fast-dcfg", TUNABLE_PLAIN, PLAIN_WORD);

    prepare_tunable(apple_dt, "/chosen", "mac-address-wifi0",      linux_dt, "/chosen", "hwaddr-wlan0", TUNABLE_PLAIN, PLAIN_BYTE);
    prepare_tunable(apple_dt, "/chosen", "mac-address-bluetooth0", linux_dt, "/chosen", "hwaddr-bt0",   TUNABLE_PLAIN, PLAIN_BYTE);
    prepare_tunable(apple_dt, "/arm-io/wlan", "module-instance",   linux_dt, "/chosen", "module-wlan0", TUNABLE_PLAIN, PLAIN_BYTE);

    configure_x8r8g8b8();

    printf("Loader complete, relocating kernel...\n");
    printf("%d\n", dt_write_dtb(linux_dt, linux_dtb, 0xa0000));
    //dt_free(linux_dt);
    printf("really done!\n");
}

void smp_main(unsigned cpu)
{
    uint64_t mpidr_el1, midr_el1;
    srread(mpidr_el1, mpidr_el1);
    srread(midr_el1, midr_el1);
    printf("CPU%d: MPIDR %016lx, MIDR %016lx\n", cpu, mpidr_el1, midr_el1);
}
