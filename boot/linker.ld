ENTRY(_start)
OUTPUT_FORMAT(elf64-x86-64)
OUTPUT_ARCH(i386:x86-64)

SECTIONS
{
    . = 0xFFFFFFFF80000000;

    .text : AT(ADDR(.text) - 0xFFFFFFFF80000000 + 0x100000)
    {
        KEEP(*(.text.boot))
        *(.text)
        *(.text.*)
        . = ALIGN(4096);
    }

    .rodata : AT(ADDR(.rodata) - 0xFFFFFFFF80000000 + 0x100000)
    {
        *(.rodata)
        *(.rodata.*)
        . = ALIGN(4096);
    }

    .data : AT(ADDR(.data) - 0xFFFFFFFF80000000 + 0x100000)
    {
        *(.data)
        *(.data.*)
        . = ALIGN(4096);
    }

    .bss : AT(ADDR(.bss) - 0xFFFFFFFF80000000 + 0x100000)
    {
        *(COMMON)
        *(.bss)
        *(.bss.*)
        . = ALIGN(4096);
    }

    /DISCARD/ :
    {
        *(.comment)
        *(.note*)
    }
}
