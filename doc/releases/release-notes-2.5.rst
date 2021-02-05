:orphan:

.. _zephyr_2.5:

Zephyr 2.5.0 (Working Draft)
############################

We are pleased to announce the release of Zephyr RTOS version 2.5.0.

Major enhancements with this release include:

* Introduced support for the SPARC processor architecture and the LEON
  processor implementation.

The following sections provide detailed lists of changes by component.

Security Vulnerability Related
******************************

The following CVEs are addressed by this release:

More detailed information can be found in:
https://docs.zephyrproject.org/latest/security/vulnerabilities.html

Known issues
************

You can check all currently known issues by listing them using the GitHub
interface and listing all issues with the `bug label
<https://github.com/zephyrproject-rtos/zephyr/issues?q=is%3Aissue+is%3Aopen+label%3Abug>`_.

API Changes
***********

* Removed SETTINGS_USE_BASE64 support as its been deprecated for more than
  two releases.

* The :c:func:`lwm2m_rd_client_start` function now accepts an additional
  ``flags`` parameter, which allows to configure current LwM2M client session,
  for instance enable bootstrap procedure in the curent session.

* LwM2M execute now supports arguments. The execute callback
  `lwm2m_engine_execute_cb_t` is extended with an ``args`` parameter which points
  to the CoAP payload that comprises the arguments, and an ``args_len`` parameter
  to indicate the length of the ``args`` data.

* Changed vcnl4040 dts binding default for property 'proximity-trigger'.
  Changed the default to match the HW POR state for this property.

* The :c:func:`clock_control_async_on` function will now take ``callback`` and
  ``user_data`` as arguments instead of structure which contained list node,
  callback and user data.

* The :c:func:`mqtt_keepalive_time_left` function now returns -1 if keep alive
  messages are disabled by setting ``CONFIG_MQTT_KEEPALIVE`` to 0.

* The ``CONFIG_LEGACY_TIMEOUT_API`` mode has been removed.  All kernel
  timeout usage must use the new-style k_timeout_t type and not the
  legacy/deprecated millisecond counts.

* The :c:func:`coap_pending_init` function now accepts an additional ``retries``
  parameter, allowing to specify the maximum retransmission count of the
  confirmable message.

* The ``CONFIG_BT_CTLR_CODED_PHY`` is now disabled by default for builds
  combining both Bluetooth host and controller.

* The :c:func:`coap_packet_append_payload` function will now take a pointer to a
  constant buffer as the ``payload`` argument instead of a pointer to a writable
  buffer.

* The :c:func:`coap_packet_init` function will now take a pointer to a constant
  buffer as the ``token`` argument instead of a pointer to a writable buffer.

* A new :ref:`regulator_api` API has been added to support controlling power
  sources.  Regulators can also be associated with devicetree nodes, allowing
  drivers to ensure the device they access has been powered up.  For simple
  GPIO-only regulators a devicetree property ``supply-gpios`` is defined as a
  standard way to identify the control signal in nodes that support power
  control.

* :c:type:`fs_tile_t` objects must now be initialized by calling
  :c:func:`fs_file_t_init` before their first use.

Deprecated in this release
==========================

* Nordic nRF5340 PDK board deprecated and planned to be removed in 2.6.0.
* ARM Musca-A board and SoC support deprecated and planned to be removed in 2.6.0.

* DEVICE_INIT was deprecated in favor of utilizing DEVICE_DEFINE directly.

* DEVICE_AND_API_INIT was deprecated in favor of DEVICE_DT_INST_DEFINE and
  DEVICE_DEFINE.

* Bluetooth

  * Deprecated the :c:func:`bt_set_id_addr` function, use :c:func:`bt_id_create`
    before calling :c:func:`bt_enable` instead. When ``CONFIG_PRIVACY`` is
    enabled a valid IRK has to be supplied by the application for this case.

Removed APIs in this release
============================

* Bluetooth

  * The deprecated BT_LE_SCAN_FILTER_DUPLICATE define has been removed,
    use BT_LE_SCAN_OPT_FILTER_DUPLICATE instead.
  * The deprecated BT_LE_SCAN_FILTER_WHITELIST define has been removed,
    use BT_LE_SCAN_OPT_FILTER_WHITELIST instead.
  * The deprecated bt_le_scan_param::filter_dup argument has been removed,
    use bt_le_scan_param::options instead.
  * The deprecated bt_conn_create_le() function has been removed,
    use bt_conn_le_create() instead.
  * The deprecated bt_conn_create_auto_le() function has been removed,
    use bt_conn_le_create_auto() instead.
  * The deprecated bt_conn_create_slave_le() function has been removed,
    use bt_le_adv_start() instead with bt_le_adv_param::peer set to the remote
    peers address.
  * The deprecated BT_LE_ADV_* macros have been removed,
    use the BT_GAP_ADV_* enums instead.
  * The deprecated bt_conn_security function has been removed,
    use bt_conn_set_security instead.
  * The deprecated BT_SECURITY_* defines NONE, LOW, MEDIUM, HIGH, FIPS have been
    removed, use the L0, L1, L2, L3, L4 defines instead.
  * The deprecated BT_HCI_ERR_AUTHENTICATION_FAIL define has been removed,
    use BT_HCI_ERR_AUTH_FAIL instead.

* Kernel

  * The deprecated k_mem_pool API has been removed entirely (for the
    past release it was backed by a k_heap, but maintained a
    compatible API).  Now all instantiated heaps must be
    sys_heap/k_heaps.  Note that the new-style heap is a general
    purpose allocator and does not make the same promises about block
    alignment/splitting.  Applications with such requirements should
    look at porting their logic, or perhaps at the k_mem_slab utility.

Stable API changes in this release
==================================

Kernel
******

Architectures
*************

* ARC

* ARM

  * AARCH32

    * Introduced the functionality for chain-loadable Zephyr
      fimrmware images to force the initialization of internal
      architecture state during early system boot (Cortex-M).
    * Changed the default Floating Point Services mode to
      Shared FP registers mode.

  * AARCH64

* POSIX

* RISC-V

* SPARC

  * Added support for the SPARC architecture, compatible with the SPARC V8
    specification and the SPARC ABI.
  * FPU is supported in both shared and unshared FP register mode.

* x86

Boards & SoC Support
********************

* Added support for these SoC series:

  * Cypress PSoC-63

* Made these changes in other SoC series:

* Changes for ARC boards:

* Added support for these ARM boards:

  * Cypress CY8CKIT_062_BLE board

* Added support for these SPARC boards:

  * GR716-MINI LEON3FT microcontroller development board
  * Generic LEON3 board configuration for GRLIB FPGA reference designs
  * SPARC QEMU for emulating LEON3 processors and running kernel tests

* Added support for these STM32 boards and SoCs :

  * Cortex-M Trace Reference Board V1.2 (SEGGER TRB STM32F407)
  * MikroE Clicker 2 for STM32
  * STM32F103RCT6 Mini
  * ST Nucleo F303K8
  * ST Nucleo F410RB
  * ST Nucleo H723ZG
  * ST Nucleo L011K4
  * ST Nucleo L031K6
  * ST Nucleo L433RC-P
  * ST STM32L562E-DK Discovery
  * STM32F105xx and STM32F103xG SoC variants
  * STM32G070xx SoC variants
  * STM32G474xB/C SoC variants
  * STM32L071xx SoC variants
  * STM32L151xC and STM32L152xC SoC variants

* Made these global changes in STM32 boards and SoC series:

  * Pin control configuration is now done through device tree and existing
    macros to configure pins in pinmux.c files are tagged as deprecated.
    The new pin settings are provided thanks to .dtsi files distributed in
    hal_stm32 module.
  * Generic LL headers, also distributed in hal_stm32 module, are now available
    to abstract series references in drivers.
  * Hardware stack protection is now default on all boards with enabled MPU
    (SRAM > 64K ), excluding F0/G0/L0 series.
  * West flash STM32CubeProgrammer runner was added as a new option for STM32
    boards flashing (to be installed separately).

* Made these changes in other boards:

  * CY8CKIT_062_WIFI_BT_M0: was renamed to CY8CKIT_062_WIFI_BT.
  * CY8CKIT_062_WIFI_BT_M4: was moved into CY8CKIT_062_WIFI_BT.
  * CY8CKIT_062_WIFI_BT: Now M0+/M4 are at same common board.
  * nRF5340 DK: Selected TF-M as the default Secure Processing Element
    (SPE) when building Zephyr for the non-secure domain.
  * SAM4E_XPRO: Added support to SAM-BA ROM bootloader.
  * SAM4S_XPLAINED: Added support to SAM-BA ROM bootloader.

* Added support for these following shields:

  * Inventek es-WIFI shield
  * Sharp memory display generic shield

Drivers and Sensors
*******************

* ADC

* Audio

* Bluetooth

* CAN

  * We reworked the configuration API.
    A user can now specify the timing manually (define prop segment,
    phase segment1, phase segment2, and prescaler) or use a newly introduced
    algorithm to calculate optimal timing values from a bitrate and sample point.
    The bitrate and sample point can be specified in the device tree too.
    It is possible to change the timing values at runtime now.

  * We reworked the zcan_frame struct due to undefined behavior.
    The std_id (11-bit) and ext_id (29-bit) are merged to a single id
    field (29-bit). The union of both IDs was removed.

  * We made the CANbus API CAN-FD compatible.
    The zcan_frame data-field can have a size of >8 bytes now.
    A flag was introduced to mark a zcan_frame as CAN-FD frame.
    A flag was introduced that enables a bitrate switch in CAN-FD frames.
    The configuration API supports an additional timing parameter for the CAN-FD
    data-phase.

  * drivers are converted to use the new DEVICE_DT_* macros.

* Clock Control

* Console

* Counter

* Crypto

* DAC

* Debug

* Display

* DMA

  * kmalloc was removed from STM32 DMAMUX driver initialization.

* EEPROM

  * Marked the EEPROM API as stable.
  * Added support for AT24Cxx devices.

* Entropy

* ESPI

* Ethernet

  * Added support for Distributed Switch Architecture (DSA) devices.
    Currently only ip_k66f board supports DSA.
  * Added support for w5500 Ethernet controller.

* Flash

  * CONFIG_NORDIC_QSPI_NOR_QE_BIT has been removed.  The
    quad-enable-requirements devicetree property should be used instead.
  * MPU_ALLOW_FLASH_WRITE is now default on STM32 boards when MPU is enabled.
  * Add driver for STM32H7 and STM32L1 SoC series.
  * Add QSPI NOR Flash controller support for STM32 family.

* GPIO

  * Added Cypress PSoC-6 driver.
  * Added Atmel SAM4L driver.

* Hardware Info

  * Added Cypress PSoC-6 driver.

* I2C

  * Added Atmel SAM4L TWIM driver.

* I2S

* IEEE 802.15.4

  * nRF:

    * Added IEEE 802.15.4 support for nRF5340.
    * Added support for failed rx notification.

  * cc13xx/cc26xx:

    * Added multi-protocol radio support.
    * Added sub-ghz support.
    * Added raw mode support.

* Interrupt Controller

  * Added Cypress PSoC-6 Cortex-M0+ interrupt multiplexer driver.

* IPM

* Keyboard Scan

* LED

* LED Strip

* LoRa

* memc

  * Added FMC/SDRAM memory controller for STM32 family

* Modem

  * Improved RX with HW flow control in modem interface API.
  * Improved reading from interface in command handler.
  * Fixed race condition when waiting on cmd reply.
  * Added support for Quectel bg95 modem.
  * Constified modem command structures to reduce RAM usage.

  * hl7800:

    * Fixed buffer handling issues.
    * Fixed setting DNS address.
    * Fixed file open in fw update.
    * Fixed cases where socket would not close.

  * sara-r4:

    * Added sanity timeout for @ prompt.
    * Fixed redundant wait after sendto.
    * Improved offload_sendmsg() support.
    * Added Kconfig to configure RSSI work.
    * Added direct CMD to catch @ when sending data.
    * Sanitize send_socket_data() semaphore handling.

  * bg96:

    * Fixed UDP packet management.

  * GSM:

    * Added start/stop API support so that application can turn off
      the GSM/PPP modem if needed to save power.
    * Avoid wrapping each byte in muxing headers in PPP.
    * Added support to remove PPP IPv4 ipcp address on network down.

* PECI

* Pinmux

  * STM32 pinmux driver has been reworked to allow pin configuration using
    device tree definitions. The previous C macros are now deprecated.

* PS/2

* PWM

* Sensor

* Serial

  * Added ASYNC API support on STM32 family.

* SPI

* Timer

* USB

  * Made USB DFU class compatible with the target configuration that does not
    have a secondary image slot.
  * Support to use USB DFU within MCUBoot with single application slot mode.

* Video

* Watchdog

* WiFi

  * eswifi:

    * Added uart bus interface. This enables all Inventek modules with
      IWIN AT Commands firmware.

  * esp:

    * Fixed thread-safety access on esp_socket operations.
    * Fixed scheduling each RX packet on separate work thread.
    * Fixed initializing socket work structures only once.
    * Reworked +IPD and +CIPRECVDATA handling.
    * Stopped locking scheduler when sending data.
    * Added DHCP/Static IP Support.
    * Added support using DNS servers.
    * Enhanced CWMODE support.
    * Added support for configuring hostname.
    * Added support for power-gpios to enable ESP module.
    * Added support 32-bit length in +IPD.
    * Added support for reconfiguring UART baudrate after initial communication.
    * Improved packet allocation failure handling by closing stream sockets.

Networking
**********

* CoAP:

  * Fixed discovery response formatting according to RFC6690.
  * Randomized initial ACK timeout.
  * Reworked pending retransmission logic.
  * Fixed long options encoding.

* DHCPv4:

  * Added start/bound/stop network management events for DHCPv4.
  * Fixed timeout scheduling with multiple network interfaces.
  * Fixed timeout on entry to bound state.
  * Fixed invalid timeout on send failure.
  * Fixed bounds checking in timeout.
  * Fixed endian issue.
  * Added randomization to message interval.
  * Limited message interval to a maximum of 64 seconds.

* DNS:

  * Added resolving literal IP addresses even when DNS is disabled.
  * Added support for DNS Service Discovery (dns-sd).
  * Fixed getaddrinfo() to respect socket type hints.

* HTTP:

  * Added chunked encoding body support to HTTP client API.

* IPv6:

  * Tweaked IPv6 DAD and RS timeout handling.
  * Fixed multiple endian issues.
  * Fixed unaligned access to IPv6 address.

* LwM2M:

  * Added dimension discovery support.
  * Implemented bootstrap discovery.
  * Fixed message find based on pending/reply.
  * Reworked bootstrap DELETE operation.
  * Added path generation macro.
  * Added a way to notify the application on network error.
  * Added a callback to notify socket errors to applications.
  * Send Registration Update on lifetime changes.
  * Fixed PULL FW update in case of URI parse errors.
  * Fixed separate response handling.
  * Start notify sequence numbers on 0.
  * Enhanced packing of TLV integers more efficiently.
  * Improved token generation.
  * Fixed the bootstrap to be optional.

* Misc:

  * Allow user to select pre-emptive or co-operative RX/TX threads.
  * Refactored RX and TX thread priorities.
  * Only start the network logging backend if the autostarting is enabled.
  * Added support for simultaneous UDP/TCP and raw sockets in applications.
  * Enabled solicit node multicast group registration for Bluetooth IPSP
    connections.
  * Added net_buf_remove API to manipulate data at the end of network buffers.
  * Added checks to syslog-net that ensure immediate logging mode is not set as
    the network logging is not compatible with it.
  * Implemented SO_RCVTIMEO socket receive timeout option.
  * Added support to update unique hostname on link address changes.
  * Added locking to IPv6, CAN and packet socket bind calls.
  * Added network management events monitor support.

* MQTT:

  * Reset client state before notifying application with MQTT_EVT_DISCONNECT event.

* OpenThread:

  * Added support for RCP (Radio Co-Processor) mode.
  * Made radio workqueue stack size configurable.
  * Added joining thread multicast addresses which are added to Zephyr.
  * Added SRP Kconfig options.
  * Enabled CSL and TREL config options.
  * Added option to enable software CSMA backoff.
  * Added support to configure platform info.
  * Added Kconfigs to change values in Zephyr.
  * Removed unused defines from platform configuration.

* Samples:

  * Added TagoIO IoT Cloud HTTP post sample.
  * Fixed the return code in MQTT Docker tests.
  * Added support to allow DHCPv4 or manually set addresses in zperf sample.
  * Use IPv4 instead of IPv6 in coap-server to support Docker based testing.
  * Added connection manager support to dumb_http_server_mt sample.
  * Added support for large file in dumb_http_server_mt sample.
  * Added support for running the gptp sample X seconds to support Docker based testing.
  * Added Docker based testing to http_client sample.
  * Refractored code structure and reduced RAM usage of civetweb sample.
  * Added suspend/resume shell commands to gsm_modem sample.
  * Added Docker based testing support to network logging sample.

* TCP:

  * The new TCP stack is enabled by default. Legacy TCP stack is deprecated but
    still available and scheduled for removal in next 2.6 release.
  * Added support to queue received out-of-order TCP data.
  * Added connection termination if the TCP handshake is not finalized.
  * Enhanced received TCP RST packet handling.
  * Fixed TCP connection from Windows 10.

* TLS:

  * Use Maximum Fragment Length (MFL) extension by default.
  * Added ALPN extension option to TLS.
  * Fixed TLS context leak on socket allocation failure.

Bluetooth
*********

* Host

  * When privacy has been enabled in order to advertise towards a
    privacy-enabled peer the BT_LE_ADV_OPT_DIR_ADDR_RPA option must now
    be set, same as when privacy has been disabled.

* Mesh

  * The ``bt_mesh_cfg_srv`` structure has been deprecated in favor of a
    standalone Heartbeat API and Kconfig entries for default state values.


* BLE split software Controller

* HCI Driver

Build and Infrastructure
************************

* Improved support for additional toolchains:

* Devicetree

  * :c:macro:`DT_ENUM_IDX_OR`: new macro
  * Support for legacy devicetree macros via
    ``CONFIG_LEGACY_DEVICETREE_MACROS`` was removed. All devicetree-based code
    should be using the new devicetree API introduced in Zephyr 2.3 and
    documented in :ref:`dt-from-c`. Information on flash partitions has moved
    to :ref:`flash_map_api`.
  * It is now possible to resolve at build time the device pointer associated
    with a device that is defined in devicetree, via ``DEVICE_DT_GET``.  See
    :ref:`dt-get-device`.

* West

  * Improve bossac runner. It supports now native ROM bootloader for Atmel
    MCUs and extended SAM-BA bootloader like Arduino and Adafruit UF2. The
    devices supported depend on bossac version inside Zephyr SDK or in users
    path. The recommended Zephyr SDK version is 0.12.0 or newer.

Libraries / Subsystems
**********************

* File systems

  * API

    * Added c:func:`fs_file_t_init` function for initialization of
      c:type:`fs_file_t` objects.

* Disk

* File Systems

  * :option:`CONFIG_FS_LITTLEFS_FC_MEM_POOL` has been deprecated and
    should be replaced by :option:`CONFIG_FS_LITTLEFS_FC_HEAP_SIZE`.

* Management

  * MCUmgr

    * Added support for flash devices that have non-0xff erase value.
    * Added optional verification, enabled via
      :option:`CONFIG_IMG_MGMT_REJECT_DIRECT_XIP_MISMATCHED_SLOT`, of an uploaded
      Direct-XIP binary, which will reject any binary that is not able to boot
      from base address of offered upload slot.

  * updatehub

    * Added support to Network Manager and interface overlays at UpdateHub
      sample. Ethernet is the default interface configuration and overlays
      can be used to change default configuration
    * Added WIFI overlay
    * Added MODEM overlay
    * Added IEEE 802.15.4 overlay [experimental]
    * Added BLE IPSP overlay as [experimental]
    * Added OpenThread overlay as [experimental].

* Settings

* Random

* POSIX subsystem

* Power management

* Logging

* LVGL

  * Library has been updated to minor release v7.6.1

* Shell

* Storage

  * flash_map: Added API to get the value of an erased byte in the flash_area,
    see ``flash_area_erased_val()``.

* Tracing

* Debug

* DFU

 * boot: Reworked using MCUBoot's bootutil_public library which allow to use
   API implementation already provided by MCUboot codebase and remove
   zephyr's own implementations.

HALs
****

* HALs are now moved out of the main tree as external modules and reside in
  their own standalone repositories.

MCUBoot
*******

* bootloader

  * Added hardening against hardware level fault injection and timing attacks,
    see ``CONFIG_BOOT_FIH_PROFILE_HIGH`` and similar kconfig options.
  * Introduced Abstract crypto primitives to simplify porting.
  * Added ram-load upgrade mode (not enabled for zephy-rtos yet).
  * Renamed single-image mode to single-slot mode,
    see ``CONFIG_SINGLE_APPLICATION_SLOT``.
  * Added patch for turning off cache for Cortex M7 before chain-loading.
  * Fixed boostrapping in swap-move mode.
  * Fixed issue causing that interrupted swap-move operation might brick device
    if the primary image was padded.
  * Fixed issue causing that HW stack protection catches the chain-loaded
    application during its early initialization.
  * Added reset of Cortex SPLIM registers before boot.
  * Fixesd build issue that occurs if CONF_FILE contains multiple file paths
    instead of single file path.
  * Added watchdog feed on nRF devices. See ``CONFIG_BOOT_WATCHDOG_FEED`` option.
  * Removed the flash_area_read_is_empty() port implementation function.
  * Initialize the ARM core configuration only when selected by the user,
    see ``CONFIG_MCUBOOT_CLEANUP_ARM_CORE``.
  * Allow the final data chunk in the image to be unaligned in
    the serial-recovery protocol.
  * Kconfig: allow xip-revert only for xip-mode.
  * ext: tinycrypt: update ctr mode to stream.
  * Use minimal CBPRINTF implementation.
  * Configure logging to LOG_MINIMAL by default.
  * boot: cleanup NXP MPU configuration before boot.
  * Fix nokogiri<=1.11.0.rc4 vulnerability.
  * bootutil_public library was extracted as code which is common API for
    MCUboot and the DFU application, see ``CONFIG_MCUBOOT_BOOTUTIL_LIB``

* imgtool

  * Print image digest during verify.
  * Add possibility to set confirm flag for hex files as well.
  * Usage of --confirm implies --pad.
  * Fixed 'custom_tlvs' argument handling.
  * Add support for setting fixed ROM address into image header.
  * Fixed verification with protected TLVs.


Trusted-Firmware-M
******************

* Synchronized Trusted-Firmware-M module to the upstream v1.2.0 release.

Documentation
*************

Tests and Samples
*****************

  * A sample was added to demonstrate how to use the ADC driver API.

Issue Related Items
*******************

These GitHub issues were addressed since the previous 2.4.0 tagged
release:
