	.text

	.global boot_helper

boot_helper:
#ifdef WITH_DTB
	csrr a0, mhartid
	la a1, dt_blob_start
#else
	sd a0, _a0, t0
	sd a1, _a1, t0
	sd a2, _a2, t0
#endif
	sd a3, _ad, t0
	li a4, 1
	sd a4, _st, t0
	jr a3

	.global hart_wait

hart_wait:
	sd zero, _st, t0
hart_wait_loop:
	ld t0, _st
	beqz t0, hart_wait_loop
#ifdef WITH_DTB
	csrr a0, mhartid
	la a1, dt_blob_start
#else
	ld a0, _a0
	ld a1, _a1
	ld a2, _a2
#endif
	ld a3, _ad
	jr a3

	.data

	.comm _st,8,8
	.comm _ad,8,8
	.comm _a0,8,8
	.comm _a1,8,8
	.comm _a2,8,8

#ifdef WITH_DTB
dt_blob_start:
	.incbin "generated/devicetree.dtb"
#endif

