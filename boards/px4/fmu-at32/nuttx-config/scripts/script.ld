/****************************************************************************
 * configs/omnibus-f4sd/scripts/ld.script
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* The STM32F405RG has 1024Kb of FLASH beginning at address 0x0800:0000 and
 * 192Kb of SRAM. SRAM is split up into three blocks:
 *
 * 1) 112Kb of SRAM beginning at address 0x2000:0000
 * 2)  16Kb of SRAM beginning at address 0x2001:c000
 * 3)  64Kb of CCM SRAM beginning at address 0x1000:0000
 *
 * When booting from FLASH, FLASH memory is aliased to address 0x0000:0000
 * where the code expects to begin execution by jumping to the entry point in
 * the 0x0800:0000 address range.
 *
 * The first 16 KiB of flash is reserved for the bootloader.
 * Paramater storage will use the next 16KiB Sector.
 */

MEMORY
{
    flash (rx)   : ORIGIN = 0x08040000, LENGTH = 3776K
    flash2 (rx)   : ORIGIN = 0x08004000, LENGTH = 240K
    sram (rwx)   : ORIGIN = 0x20000000, LENGTH = 256K

}

OUTPUT_ARCH(arm)

ENTRY(__start)		/* treat __start as the anchor for dead code stripping */
EXTERN(_vectors)	/* force the vectors to be included in the output */

/*
 * Ensure that abort() is present in the final object.  The exception handling
 * code pulled in by libgcc.a requires it (and that code cannot be easily avoided).
 */
EXTERN(abort)

SECTIONS
{
/*
	.quick : {
	/*	. = ALIGN(4);
		*modules__mc_att_control.a:*(.text* .rodata*)
		*modules__mc_rate_control.a:*(.text* .rodata*)
		*drivers__imu__invensense__icm42688p.a:*(.text* .rodata*)
		*drivers__device.a:*(.text* .rodata*)
		*drivers__pwm_out.a:*(.text* .rodata*)
		*vehicle_angular_velocity.a:*(.text* .rodata*)
		*modules__attitude_estimator_q.a:*(.text* .rodata*)
		*at32_spi.o(.text*)
		*at32_dma.o(.text* .rodata*)
		*modules__flight_mode_manager.a:*(.text* .rodata*)
		*modules__mc_pos_control.a:*(.text* .rodata*)
		*modules__mc_hover_thrust_estimator.a:*(.text* .rodata*)
		*modules__rc_update.a:*(.text* .rodata*)
		*modules__sensors.a:*(.text* .rodata*)
		*libc.a:*(.text* .rodata*)
		*modules__commander.a:*(.text* .rodata*)
		*modules__dataman.a:*(.text* .rodata*)
		*modules__events.a:*(.text* .rodata*)
		*modules__load_mon.a:*(.text* .rodata*)
	/*	*modules__logger.a:*(.text* .rodata*)
		*battery_status.a:*(.text* .rodata*)
		*land_detector.a:*(.text* .rodata*)
		*uorb_msgs.a:*(.text* .rodata*)
		*arch_spi.a:*(.text* .rodata*)
	/*	*modules__mavlink.a:*(.text* .rodata*)
	/*	*px4_work_queue.a:*(.text* .rodata*)
		*(.quick)
		*(.quick.*)
	/*	. = ALIGN(4);
		FILL(0)
	} > flash2
*/

	.quick : {
		. = ALIGN(4);
		*px4_work_queue.a:*(.rodata*)
		*modules__mc_att_control.a:*(.text*)
		*modules__mc_rate_control.a:*(.text*)
		*drivers__imu__invensense__icm42688p.a:*(.text*)
		*drivers__device.a:*(.text*)
		*drivers__pwm_out.a:*(.text*)
		*vehicle_angular_velocity.a:*(.text*)
		*modules__attitude_estimator_q.a:*(.text*)
		*spi.o(.text*)
		*at32_dma.o(.text*)
		*modules__flight_mode_manager.a:*(.text*)
		*modules__mc_pos_control.a:*(.text*)
		*modules__mc_hover_thrust_estimator.a:*(.text*)
		*modules__rc_update.a:*(.text*)
		*modules__sensors.a:*(.text*)
		*libc.a:*(.text*)
		*modules__commander.a:*(.text*)
		*modules__dataman.a:*(.text*)
		*modules__events.a:*(.text*)
		*modules__load_mon.a:*(.text*)
		*modules__logger.a:*(.text*)
		*battery_status.a:*(.text*)
		*land_detector.a:*(.text*)
		*uorb_msgs.a:*(.text*)
		*arch_spi.a:*(.text*)
		*(.quick)
		*(.quick.*)
		. = ALIGN(4);
	} > flash2

	.text : {
		_stext = ABSOLUTE(.);
		KEEP(*(.vectors))
		*(.text .text.*)
		*(.fixup)
		*(.gnu.warning)
		*(.rodata .rodata.*)
		*(.gnu.linkonce.t.*)
		*(.got)
		*(.gcc_except_table)
		*(.gnu.linkonce.r.*)
		_etext = ABSOLUTE(.);
		FILL(0)
	} > flash

	/*
	 * Init functions (static constructors and the like)
	 */
        .init_section : {
                _sinit = ABSOLUTE(.);
                KEEP(*(.init_array .init_array.*))
                _einit = ABSOLUTE(.);
        } > flash


	.ARM.extab : {
		*(.ARM.extab*)
	} > flash

	__exidx_start = ABSOLUTE(.);
	.ARM.exidx : {
		*(.ARM.exidx*)
	} > flash
	__exidx_end = ABSOLUTE(.);

	_eronly = ABSOLUTE(.);

	.data : {
		_sdata = ABSOLUTE(.);
		*(.data .data.*)
		*(.gnu.linkonce.d.*)
		CONSTRUCTORS
		_edata = ABSOLUTE(.);
	} > sram AT > flash

	.bss : {
		_sbss = ABSOLUTE(.);
		*(.bss .bss.*)
		*(.gnu.linkonce.b.*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = ABSOLUTE(.);
	} > sram




	/* Stabs debugging sections. */
	.stab 0 : { *(.stab) }
	.stabstr 0 : { *(.stabstr) }
	.stab.excl 0 : { *(.stab.excl) }
	.stab.exclstr 0 : { *(.stab.exclstr) }
	.stab.index 0 : { *(.stab.index) }
	.stab.indexstr 0 : { *(.stab.indexstr) }
	.comment 0 : { *(.comment) }
	.debug_abbrev 0 : { *(.debug_abbrev) }
	.debug_info 0 : { *(.debug_info) }
	.debug_line 0 : { *(.debug_line) }
	.debug_pubnames 0 : { *(.debug_pubnames) }
	.debug_aranges 0 : { *(.debug_aranges) }
}
