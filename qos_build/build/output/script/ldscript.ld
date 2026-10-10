ENTRY(qos_app_preboot)
MEMORY
{
  MSMB_AREA(rwx) : ORIGIN = ((0x0c400000) - (0x100000)), LENGTH = (((0x0c400000)) - ((0x0c400000) - (0x100000)))
  FLASH_AREA(rx) : ORIGIN = (((0x00800000) + (((0x82000)) + (0x1D6000)))), LENGTH = (((0xC8000)))
}
SECTIONS
{
  . = (((0x00800000) + (((0x82000)) + (0x1D6000))));
  .unirtos_app_entry :
  {
    KEEP(*(.unirtos_app_entry))
  } >FLASH_AREA
  .LOAD_APP_TABLE ((((0x00800000) + (((0x82000)) + (0x1D6000)))) + 0x80):
  {
    . = ALIGN(4);
    Load$$LOAD_APP_TABLE$$Base = LOADADDR(.LOAD_APP_TABLE);
    Image$$LOAD_APP_TABLE$$Base = .;
    KEEP(*(.LOAD_APP_TABLE*))
   Image$$LOAD_APP_TABLE$$Limit = .;
  } >FLASH_AREA
  Image$$LOAD_APP_TABLE$$Length = SIZEOF(.LOAD_APP_TABLE);
  .unirtos_app_init :
  {
    . = ALIGN(4);
    __unirtos_app_init_start = .;
    KEEP(*(.unirtos_app_init*))
    __unirtos_app_init_end = .;
    . = ALIGN(4);
  } >FLASH_AREA
  .load_ROM_FLASH :
  {
    . = ALIGN(4);
    *(.rodata*)
    *(.text*)
    . = ALIGN(4);
  } >FLASH_AREA
  .load_draw_rw_cstomer_data(((0x0c400000) - (0x100000))) :
  {
    . = ALIGN(4);
    Load$$LOAD_OCPU_CSTOMER_RAM_RW$$Base = LOADADDR(.load_draw_rw_cstomer_data);
    Image$$LOAD_OCPU_CSTOMER_RAM_RW$$Base = .;
    KEEP(*(.unirtos_app_ramcode))
    Image$$LOAD_OCPU_CSTOMER_RAM_RW$$Limit = .;
    . = ALIGN(4);
  } >MSMB_AREA AT>FLASH_AREA
  Image$$LOAD_OCPU_CSTOMER_RAM_RW$$Length = SIZEOF(.load_draw_rw_cstomer_data);
  .load_dram_rw_data :
  {
    . = ALIGN(4);
    Load$$LOAD_OCPU_RAM_RW$$Base = LOADADDR(.load_dram_rw_data);
    Image$$LOAD_OCPU_RAM_RW$$Base = .;
    *(.data*)
    . = ALIGN(4);
  } >MSMB_AREA AT>FLASH_AREA
  Image$$LOAD_OCPU_RAM_RW$$Length = SIZEOF(.load_dram_rw_data);
  .load_dram_rw_data_zi :
  {
    . = ALIGN(4);
    Image$$LOAD_OCPU_RAM_ZI$$Base = .;
    *(.bss*)
    *(.COMMON*)
    . = ALIGN(4);
  } >MSMB_AREA AT>FLASH_AREA
  Image$$LOAD_OCPU_RAM_ZI$$Length = SIZEOF(.load_dram_rw_data_zi);
  PROVIDE(end_app_data = . );
  Image$$LOAD_OCPU_RAM$$Base = ((0x0c400000) - (0x100000));
  Image$$LOAD_OCPU_RAM$$Limit = end_app_data;
}
GROUP(
    libgcc.a
    libc.a
    libm.a
 )
