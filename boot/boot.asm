[bits 16]
[org 0x7c00]

%define STAGE2_LOAD_SEGMENT     0x07e0
%define STAGE2_LOAD_OFFSET      0x0000
%define STAGE2_START_SECTOR     2
%define STAGE2_TOTAL_SECTORS    64
%define BOOT_DRIVE_NUMBER       0x80
%define STACK_BASE              0x7c00
%define DISK_EXTENSION_CHECK    0x55aa
%define DAP_STRUCTURE_SIZE      0x10
%define SECTOR_SIZE_BYTES       512
%define MAX_RETRY_COUNT         5
%define VIDEO_MODE_TEXT         0x03
%define DISK_RESET_COMMAND      0x00
%define DISK_READ_EXT_COMMAND   0x42

section .text
global _start

_start:
    cli
    xor ax, ax
    mov ds, ax
    mov es, ax
    mov fs, ax
    mov gs, ax
    mov ss, ax
    mov sp, STACK_BASE
    mov bp, STACK_BASE
    sti

    mov [boot_drive_number], dl

    mov ah, 0x00
    mov al, VIDEO_MODE_TEXT
    int 0x10

    mov si, bootloader_message
    call display_string_bios

    call verify_disk_extension_support
    call load_stage2_bootloader
    call verify_stage2_signature

    jmp STAGE2_LOAD_SEGMENT:STAGE2_LOAD_OFFSET

verify_disk_extension_support:
    mov ah, 0x41
    mov bx, DISK_EXTENSION_CHECK
    int 0x13
    jc disk_extension_error
    cmp bx, 0xaa55
    jne disk_extension_error
    ret

load_stage2_bootloader:
    mov si, disk_address_packet
    mov word [si], DAP_STRUCTURE_SIZE
    mov word [si + 2], STAGE2_TOTAL_SECTORS
    mov word [si + 4], STAGE2_LOAD_OFFSET
    mov word [si + 6], STAGE2_LOAD_SEGMENT
    mov dword [si + 8], STAGE2_START_SECTOR
    mov dword [si + 12], 0

    mov dl, [boot_drive_number]
    mov ah, DISK_READ_EXT_COMMAND
    int 0x13
    jc disk_read_error
    ret

verify_stage2_signature:
    mov ax, STAGE2_LOAD_SEGMENT
    mov es, ax
    mov di, STAGE2_LOAD_OFFSET
    add di, 510
    mov ax, [es:di]
    cmp ax, 0xaa55
    jne stage2_signature_error
    ret

display_string_bios:
    pusha
    mov ah, 0x0e
    mov bh, 0x00
    mov bl, 0x07

.display_loop:
    lodsb
    test al, al
    jz .display_done
    int 0x10
    jmp .display_loop

.display_done:
    popa
    ret

disk_extension_error:
    mov si, disk_extension_error_message
    call display_string_bios
    jmp system_halt

disk_read_error:
    mov si, disk_read_error_message
    call display_string_bios
    jmp system_halt

stage2_signature_error:
    mov si, stage2_signature_error_message
    call display_string_bios
    jmp system_halt

system_halt:
    cli
    hlt
    jmp system_halt

boot_drive_number: db 0x00

disk_address_packet:
    db 0x10
    db 0x00
    dw 0x0000
    dw 0x0000
    dw 0x0000
    dq 0x0000000000000000

bootloader_message: db "Nanokoton Bootloader Stage 1", 0x0d, 0x0a, 0x00
disk_extension_error_message: db "Disk Extensions Not Supported", 0x0d, 0x0a, 0x00
disk_read_error_message: db "Stage 2 Load Failure", 0x0d, 0x0a, 0x00
stage2_signature_error_message: db "Stage 2 Signature Invalid", 0x0d, 0x0a, 0x00

times 510 - ($ - $$) db 0x00
dw 0xaa55
