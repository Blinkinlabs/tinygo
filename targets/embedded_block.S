// Inspired by embedded_start_block.inc.S and embedded_end_block.inc.S
// Minimum viable block image from datasheet section 5.9.5.1, "Minimum Arm IMAGE_DEF"
.section .embedded_block, "a"
.p2align 2
embedded_block:
.word 0xffffded3
.word 0x10210142
.word 0x000001ff
.word 0x00000000
.word 0xab123579
embedded_block_end:
