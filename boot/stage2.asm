[bits 16]
[org 0x7e00]

%define KERNEL_LOAD_ADDRESS         0x100000
%define KERNEL_START_SECTOR         66
%define KERNEL_TOTAL_SECTORS        128
%define PROTECTED_MODE_CODE_SELECTOR 0x08
%define PROTECTED_MODE_DATA_SELECTOR 0x10
%define LONG_MODE_CODE_SELECTOR      0x28
%define LONG_MODE_DATA_SELECTOR      0x30
%define PAGE_TABLE_BASE_ADDRESS      0x1000
%define PAGE_SIZE_4KB                4096
%define PAGE_PRESENT_FLAG            0x01
%define PAGE_WRITABLE_FLAG           0x02
%define PAGE_HUGE_FLAG               0x80
%define PAGE_NO_EXECUTE_FLAG         0x8000000000000000
%define CPUID_LONG_MODE_BIT          29
%define MSR_EFER_REGISTER            0xC0000080
%define EFER_LONG_MODE_ENABLE_BIT    8
%define CR0_PROTECTED_MODE_ENABLE    0x01
%define CR4_PAE_ENABLE               0x20
%define CR0_PAGING_ENABLE            0x80000000
%define A20_TEST_ADDRESS             0x0520
%define A20_TEST_VALUE               0x34D2
%define KEYBOARD_CONTROLLER_PORT     0x64
%define KEYBOARD_DATA_PORT           0x60

section .text
global stage2_entry

stage2_entry:
    cli
    xor ax, ax
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov sp, 0x7e00
    sti

    mov si, stage2_message
    call display_string_bios

    call test_a20_gate
    call enable_a20_gate
    call verify_cpuid_support
    call verify_long_mode_support

    lgdt [global_descriptor_table_pointer]

    mov eax, cr0
    or eax, CR0_PROTECTED_MODE_ENABLE
    mov cr0, eax

    jmp PROTECTED_MODE_CODE_SELECTOR:initialize_protected_mode

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

test_a20_gate:
    push es
    mov ax, 0xffff
    mov es, ax
    mov word [0x7c00], A20_TEST_VALUE
    mov word [es:A20_TEST_ADDRESS], 0x0000
    mov ax, [0x7c00]
    cmp ax, A20_TEST_VALUE
    jne .a20_enabled
    mov si, a20_disabled_message
    call display_string_bios
    jmp system_halt

.a20_enabled:
    pop es
    ret

enable_a20_gate:
    call .keyboard_wait_input
    mov al, 0xad
    out KEYBOARD_CONTROLLER_PORT, al

    call .keyboard_wait_input
    mov al, 0xd0
    out KEYBOARD_CONTROLLER_PORT, al

    call .keyboard_wait_output
    in al, KEYBOARD_DATA_PORT
    push eax

    call .keyboard_wait_input
    mov al, 0xd1
    out KEYBOARD_CONTROLLER_PORT, al

    call .keyboard_wait_input
    pop eax
    or al, 0x02
    out KEYBOARD_DATA_PORT, al

    call .keyboard_wait_input
    mov al, 0xae
    out KEYBOARD_CONTROLLER_PORT, al

    call .keyboard_wait_input
    ret

.keyboard_wait_input:
    in al, KEYBOARD_CONTROLLER_PORT
    test al, 0x02
    jnz .keyboard_wait_input
    ret

.keyboard_wait_output:
    in al, KEYBOARD_CONTROLLER_PORT
    test al, 0x01
    jz .keyboard_wait_output
    ret

verify_cpuid_support:
    pushfd
    pop eax
    mov ecx, eax
    xor eax, 1 << 21
    push eax
    popfd
    pushfd
    pop eax
    xor eax, ecx
    jz .cpuid_not_supported
    push ecx
    popfd
    ret

.cpuid_not_supported:
    mov si, cpuid_error_message
    call display_string_bios
    jmp system_halt

verify_long_mode_support:
    mov eax, 0x80000000
    cpuid
    cmp eax, 0x80000001
    jb .long_mode_not_supported

    mov eax, 0x80000001
    cpuid
    test edx, 1 << CPUID_LONG_MODE_BIT
    jz .long_mode_not_supported
    ret

.long_mode_not_supported:
    mov si, long_mode_error_message
    call display_string_bios
    jmp system_halt

[bits 32]
initialize_protected_mode:
    mov ax, PROTECTED_MODE_DATA_SELECTOR
    mov ds, ax
    mov es, ax
    mov fs, ax
    mov gs, ax
    mov ss, ax
    mov esp, 0x90000

    mov esi, protected_mode_message
    call display_string_protected_mode

    call setup_page_tables
    call enable_long_mode
    call load_kernel_from_disk

    lgdt [global_descriptor_table_64_pointer]

    jmp LONG_MODE_CODE_SELECTOR:initialize_long_mode

display_string_protected_mode:
    pusha
    mov edx, 0xb8000
    mov ah, 0x0f

.display_loop:
    lodsb
    test al, al
    jz .display_done
    mov [edx], ax
    add edx, 2
    jmp .display_loop

.display_done:
    popa
    ret

setup_page_tables:
    mov edi, PAGE_TABLE_BASE_ADDRESS
    mov cr3, edi
    xor eax, eax
    mov ecx, 6 * PAGE_SIZE_4KB / 4
    rep stosd

    mov edi, PAGE_TABLE_BASE_ADDRESS

    ; PML4
    mov dword [edi], PAGE_TABLE_BASE_ADDRESS + 0x1000 + PAGE_PRESENT_FLAG + PAGE_WRITABLE_FLAG
    add edi, 0x1000

    ; PDP
    mov dword [edi], PAGE_TABLE_BASE_ADDRESS + 0x2000 + PAGE_PRESENT_FLAG + PAGE_WRITABLE_FLAG
    add edi, 0x1000

    ; PD - Identity map first 2GB
    mov ebx, PAGE_TABLE_BASE_ADDRESS + 0x3000
    mov ecx, 512

.setup_pd_entries:
    mov dword [edi], ebx
    add ebx, 0x1000
    add edi, 8
    loop .setup_pd_entries

    ; PT - Map first 2MB
    mov edi, PAGE_TABLE_BASE_ADDRESS + 0x3000
    mov ebx, PAGE_PRESENT_FLAG | PAGE_WRITABLE_FLAG
    mov ecx, 512

.setup_pt_entries:
    mov dword [edi], ebx
    add ebx, 0x1000
    add edi, 8
    loop .setup_pt_entries

    mov eax, cr4
    or eax, CR4_PAE_ENABLE
    mov cr4, eax
    ret

enable_long_mode:
    mov ecx, MSR_EFER_REGISTER
    rdmsr
    or eax, 1 << EFER_LONG_MODE_ENABLE_BIT
    wrmsr

    mov eax, cr0
    or eax, CR0_PAGING_ENABLE
    mov cr0, eax
    ret

load_kernel_from_disk:
    mov edi, KERNEL_LOAD_ADDRESS
    mov ebx, KERNEL_START_SECTOR
    mov ecx, KERNEL_TOTAL_SECTORS

.load_kernel_sectors:
    push ecx
    push ebx
    push edi

    mov eax, ebx
    mov ebx, edi
    call read_disk_sector_32

    pop edi
    pop ebx
    pop ecx

    add edi, SECTOR_SIZE_BYTES
    inc ebx
    loop .load_kernel_sectors
    ret

read_disk_sector_32:
    push eax
    push ebx
    push ecx
    push edx
    push edi

    mov edi, disk_address_packet_32
    mov [edi + 8], eax
    mov [edi + 4], ebx
    mov word [edi + 2], 1

    mov ah, 0x42
    mov dl, [boot_drive_number_32]
    mov si, disk_address_packet_32
    int 0x13
    jc disk_read_error_32

    pop edi
    pop edx
    pop ecx
    pop ebx
    pop eax
    ret

disk_read_error_32:
    mov esi, kernel_load_error_message
    call display_string_protected_mode
    jmp system_halt_32

system_halt_32:
    cli
    hlt
    jmp system_halt_32

[bits 64]
initialize_long_mode:
    mov ax, LONG_MODE_DATA_SELECTOR
    mov ds, ax
    mov es, ax
    mov fs, ax
    mov gs, ax
    mov ss, ax
    mov rsp, 0xffff800000000000

    mov rsi, long_mode_message
    call display_string_long_mode

    ; Verify kernel signature
    mov rax, [KERNEL_LOAD_ADDRESS + 0x18]
    cmp rax, 0x0000000002000000
    jne kernel_signature_error

    ; Jump to kernel entry point
    mov rax, [KERNEL_LOAD_ADDRESS + 0x18]
    add rax, KERNEL_LOAD_ADDRESS
    jmp rax

display_string_long_mode:
    push rdi
    push rsi
    push rcx
    mov rdi, 0xb8000
    mov ah, 0x0f

.display_loop:
    lodsb
    test al, al
    jz .display_done
    stosw
    jmp .display_loop

.display_done:
    pop rcx
    pop rsi
    pop rdi
    ret

kernel_signature_error:
    mov rsi, kernel_signature_error_message
    call display_string_long_mode
    jmp system_halt_64

system_halt_64:
    cli
    hlt
    jmp system_halt_64

[bits 32]
boot_drive_number_32: db 0x00

disk_address_packet_32:
    db 0x10
    db 0x00
    dw 0x0001
    dd 0x00000000
    dq 0x0000000000000000

global_descriptor_table:
    ; Null descriptor
    dq 0x0000000000000000

    ; 32-bit code descriptor
    dw 0xffff
    dw 0x0000
    db 0x00
    db 0x9a
    db 0xcf
    db 0x00

    ; 32-bit data descriptor
    dw 0xffff
    dw 0x0000
    db 0x00
    db 0x92
    db 0xcf
    db 0x00

    ; 64-bit code descriptor
    dw 0x0000
    dw 0x0000
    db 0x00
    db 0x9a
    db 0x20
    db 0x00

    ; 64-bit data descriptor
    dw 0x0000
    dw 0x0000
    db 0x00
    db 0x92
    db 0x00
    db 0x00

global_descriptor_table_end:

global_descriptor_table_pointer:
    dw global_descriptor_table_end - global_descriptor_table - 1
    dd global_descriptor_table

global_descriptor_table_64_pointer:
    dw global_descriptor_table_end - global_descriptor_table - 1
    dq global_descriptor_table

stage2_message: db "Nanokoton Bootloader Stage 2", 0x0a, 0x0d, 0x00
a20_disabled_message: db "A20 Gate Disabled", 0x0a, 0x0d, 0x00
cpuid_error_message: db "CPUID Not Supported", 0x0a, 0x0d, 0x00
long_mode_error_message: db "Long Mode Not Supported", 0x0a, 0x0d, 0x00
protected_mode_message: db "Protected Mode Enabled", 0x0a, 0x0d, 0x00
kernel_load_error_message: db "Kernel Load Failed", 0x0a, 0x0d, 0x00
long_mode_message: db "Long Mode Activated", 0x0a, 0x0d, 0x00
kernel_signature_error_message: db "Kernel Signature Invalid", 0x0a, 0x0d, 0x00

times 32768 - ($ - $$) db 0x00
dw 0xaa55
