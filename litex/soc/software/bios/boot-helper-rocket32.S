    .text

	.global boot_helper
boot_helper:
	csrr a0, mhartid
	la a1, dt_blob_start
	sw a3, _ad, t0
	li a4, 1
	sw a4, _st, t0
	jr a3

	.global hart_wait
hart_wait:
	sw zero, _st, t0
hart_wait_loop:
	lw t0, _st
	beqz t0, hart_wait_loop
	csrr a0, mhartid
	la a1, dt_blob_start
	lw a3, _ad
	jr a3

    .data

    .comm _st,4,4
    .comm _ad,4,4

dt_blob_start:
    .incbin "generated/devicetree.dtb"

