	.text

	.comm _st,8,8
    .comm _r3,8,8
    .comm _ad,8,8

	.global boot_helper
boot_helper:
	csrr a0, mhartid
	la a1, dt_blob_start
	sd a2, _r3, t0
	sd a3, _ad, t0
	li a4, 1
	sd a4, _st, t0
	jr a3

	.global hart_wait
hart_wait:
	sd zero, _st, t0
	csrw mie, zero
hart_wait_loop:
	ld t0, _st
	beqz t0, hart_wait_loop
	csrr a0, mhartid
	la a1, dt_blob_start
	ld a2, _r3
	ld a3, _ad
	jr a3

    .data

dt_blob_start:
    .incbin "generated/devicetree.dtb"

