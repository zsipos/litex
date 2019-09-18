	.text

	.comm _st,8,8
	.comm _r1,8,8
        .comm _r2,8,8
        .comm _r3,8,8
        .comm _ad,8,8

	.global boot_helper
boot_helper:
	sd a0, _r1, t0
	sd a1, _r2, t0
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
	ld a0, _r1
	ld a1, _r2
	ld a2, _r3
	ld a3, _ad
	jr a3

