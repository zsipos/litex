	.text

	.global boot_helper

boot_helper:
#ifdef WITH_DTB
	csrr a0, mhartid
	la a1, dt_blob_start
#else
	sw a0, _a0, t0
	sw a1, _a1, t0
	sw a2, _a2, t0
#endif
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
#ifdef WITH_DTB
	csrr a0, mhartid
	la a1, dt_blob_start
#else
	lw a0, _a0
	lw a1, _a1
	lw a2, _a2
#endif
	lw a3, _ad
	jr a3

	.data

	.comm _st,4,4
	.comm _ad,4,4
	.comm _a0,4,4
	.comm _a1,4,4
	.comm _a2,4,4

#ifdef WITH_DTB
dt_blob_start:
	.incbin "generated/devicetree.dtb"
#endif

