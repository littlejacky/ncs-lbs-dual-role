.. _peripheral_lbs:

Bluetooth: Peripheral LBS
#########################

.. contents::
   :local:
   :depth: 2

The peripheral LBS sample demonstrates how to use the :ref:`lbs_readme`.

Requirements
************

The sample supports the following development kits:

.. table-from-sample-yaml::

.. include:: /includes/tfm.txt

The sample also requires a smartphone or tablet running a compatible mobile application.
The `Testing`_ instructions refer to `nRF Connect for Mobile`_ and `nRF Blinky`_, but you can also use other similar applications, such as `nRF Toolbox`_.

.. note::
   |thingy53_sample_note|

Overview
********

You can use the sample to transmit the button state from your development kit to another device.

.. tabs::

   .. group-tab:: nRF52 and nRF53 DKs

      When connected, the sample sends the state of **Button 1** on the development kit to the connected device, such as a phone or tablet.
      The mobile application on the device can display the received button state and control the state of **LED 3** on the development kit.

   .. group-tab:: nRF54 DKs

      When connected, the sample sends the state of **Button 0** on the development kit to the connected device, such as a phone or tablet.
      The mobile application on the device can display the received button state and control the state of **LED 2** on the development kit.

You can also use this sample to control the color of the RGB LED on the nRF52840 Dongle or Thingy:53.

User interface
**************

The user interface of the sample depends on the hardware platform you are using.

.. tabs::

   .. group-tab:: nRF52 and nRF53 DKs

      LED 1:
         Blinks when the main loop is running (that is, the device is advertising) with a period of two seconds, duty cycle 50%.

      LED 2:
         Lit when the development kit is connected.

      LED 3:
         Lit when the development kit is controlled remotely from the connected device.

      Button 1:
         Send a notification with the button state: "pressed" or "released".

   .. group-tab:: nRF54 DKs

      LED 0:
         Blinks when the main loop is running (that is, the device is advertising) with a period of two seconds, duty cycle 50%.

      LED 1:
         Lit when the development kit is connected.

      LED 2:
         Lit when the development kit is controlled remotely from the connected device.

      Button 0:
         Send a notification with the button state: "pressed" or "released".

   .. group-tab:: Thingy:53

      RGB LED:
         The RGB LED channels are used independently to display the following information:

         * Red - If the main loop is running (that is, the device is advertising).
           The LED blinks with a period of two seconds, duty cycle 50%.
         * Green - If the device is connected.
         * Blue - If user set the LED using Nordic LED Button Service.

         For example, if Thingy:53 is connected over Bluetooth, the LED color toggles between green and yellow.
         The green LED channel is kept on and the red LED channel is blinking.

      Button 1:
         Send a notification with the button state: "pressed" or "released".

   .. group-tab:: nRF52840 Dongle

      Green LED:
         Blinks, toggling on/off every second, when the main loop is running and the device is advertising.

      RGB LED:
         The RGB LED channels are used independently to display the following information:

         * Red - If Dongle is connected.
         * Green - If user set the LED using Nordic LED Button Service.

      Button 1:
         Send a notification with the button state: "pressed" or "released".

Building and running
********************

.. |sample path| replace:: :file:`samples/bluetooth/peripheral_lbs`

.. include:: /includes/build_and_run_ns.txt

.. include:: /includes/nRF54H20_erase_UICR.txt

Minimal build
=============

You can build the sample with a minimum configuration as a demonstration of how to reduce code size and RAM usage, using the ``-DFILE_SUFFIX=minimal`` flag in your build.

See :ref:`cmake_options` for instructions on how to add this option to your build.
For example, when building on the command line, you can add the option as follows:

.. code-block:: console

   west build samples/bluetooth/peripheral_lbs -- -DFILE_SUFFIX=minimal

.. _peripheral_lbs_testing:

Testing
=======

After programming the sample to your dongle or development kit, one of the LEDs starts blinking to indicate that the advertising loop is active (see `User interface`_ for details).


.. tabs::

   .. group-tab:: nRF Connect for Mobile

      To test the sample using the `nRF Connect for Mobile`_ application, complete the following steps:

      .. tabs::

         .. group-tab:: nRF52 and nRF53 DKs

            1. Install and start the `nRF Connect for Mobile`_ application on your smartphone or tablet.
            #. Power on the development kit or insert your dongle into the USB port.
            #. Connect to the device from the application.
               The device is advertising as ``Nordic_LBS``.
               The services of the connected device are shown.
            #. In **Nordic LED Button Service**, enable notifications for the **Button** characteristic.
            #. Press **Button 1** on the device.
            #. Observe that notifications with the following values are displayed:

               * ``Button released`` when **Button 1** is released.
               * ``Button pressed`` when **Button 1** is pressed.

            #. Write the following values to the LED characteristic in the **Nordic LED Button Service**.
               Depending on the hardware platform, this produces results described in the table.

               +------------------------+---------+----------------------------------------------+
               | Hardware platform      | Value   | Effect                                       |
               +========================+=========+==============================================+
               | nRF52 and nRF53 DKs    | ``OFF`` | Switch the **LED 3** off.                    |
               +                        +---------+----------------------------------------------+
               |                        | ``ON``  | Switch the **LED 3** on.                     |
               +------------------------+---------+----------------------------------------------+
               | nRF52840 Dongle        | ``OFF`` | Switch the green channel of the RGB LED off. |
               +                        +---------+----------------------------------------------+
               |                        | ``ON``  | Switch the green channel of the RGB LED on.  |
               +------------------------+---------+----------------------------------------------+
               | Thingy:53              | ``OFF`` | Switch the blue channel of the RGB LED off.  |
               +                        +---------+----------------------------------------------+
               |                        | ``ON``  | Switch the blue channel of the RGB LED on.   |
               +------------------------+---------+----------------------------------------------+

         .. group-tab:: nRF54 DKs
=========================
Dual-role Nordic LBS
=========================

此工程基于官方示例“Bluetooth: Peripheral LBS”二次修改，实现了一个双角色（Peripheral + Central）的 LBS（LED Button Service）演示程序。

主要特点
---------

- 同时作为 LBS 外设（Peripheral）广播并接受来自客户端的写操作以控制本地 LED。
- 作为中央设备（Central）扫描并连接其它支持 LBS 的远端设备，发现远端 LBS 服务并订阅 Button 通知。
- 在本地按键状态变化时，向已连接的远端设备写入 LED 特征以控制远端 LED。
- 使用 Nordic DK 的按钮和 LED（通过 `dk_buttons_and_leds` 驱动）作为用户接口。

实现细节（对应 `src/main.c`）
--------------------------------

- 广播：使用 LBS 服务 UUID（BT_UUID_LBS_VAL）并以可连接模式广告。
- 扫描：启用 UUID 过滤以查找含 LBS 服务的设备（使用 `bt_scan`）。
- 连接：当作为 central 成功连接到远端设备时，会进行 GATT 服务发现；发现 LBS 后，查找 Button 特征并发现 CCC 描述符以订阅通知。
- 订阅回调：收到远端 Button 的通知会在本地打印并点亮/熄灭 USER LED。
- 本地按键：按键状态变化会更新本地状态并（如果已连接并找到了远端 LED 特征句柄）通过 GATT 写操作写入远端 LED 特征。

硬件/按键 LED 对应（在代码中）：

- RUN_LED: 运行状态指示（周期性闪烁）。
- CON_LED: 连接状态指示（连接时常亮）。
- USER_LED: 远端控制/本地显示按钮状态。
- USER_BUTTON: 本地按键输入。

快速开始（构建与运行）
------------------------

下面示例假设你已安装 NCS/Zephyr 开发环境并能够使用 `west` 和已配置的交叉工具链。

1. 选择目标开发板，例如：

    west build -b <board> -d build

    如果需要最小配置或不同的 config，可以传递额外 CMake 选项或使用仓库中的 board 配置文件。

2. 将固件写入开发板（例如通过 `west flash` 或使用 IDE）：

    west flash -d build

3. 运行：上电后设备会同时开始广播和扫描。RUN LED 会以 1s 周期闪烁表示主循环运行。

测试与验证
------------

- 使用手机应用（例如 nRF Connect）连接到本设备（会以配置的设备名广告）。
- 在 `Nordic LED Button Service` 中：
   - 对 Button 特征启用通知：按下本设备按钮会在手机上显示通知（若连接为 central，则显示远端按钮变化）。
   - 向 LED 特征写入值（例如字符串 `ON`/`OFF` 取决于客户端应用）可切换本地 USER LED（或相应硬件上的 LED 通道）。
- 当该程序作为 Central 连接到远端 LBS 设备时，会打印远端按钮通知并点亮/熄灭本地 USER LED；本地按键会向远端写 LED 状态。

配置选择
----------

- 示例使用的 Kconfig 配置位和设备名称通过仓库中的 `prj.conf` / `boards` 配置文件管理。
- 常见选项：
   - CONFIG_BT: 启用 Bluetooth
   - CONFIG_BT_LBS: 启用 Nordic LBS 服务支持
   - CONFIG_SETTINGS: 可选，启用时会加载非易失性设置

源码组织
---------

- `src/main.c`：应用主逻辑（广播、扫描、GATT 发现、订阅、按键/LED 处理）。
- `boards/`：板级配置文件（示例针对不同开发板的 conf）。
- `prj.conf`, `prj_minimal.conf`：项目配置。

故障排查要点
---------------

- 若设备未广告或扫描，检查 Bluetooth 是否成功初始化（控制台会有 "Bluetooth initialized" 日志）。
- 若未发现远端服务，确认远端设备确实在广播 LBS UUID 并且信号可达。
- GATT 写/订阅失败会在日志打印错误码，常见原因为连接断开或句柄无效。

许可证
------

见源文件头部 SPDX 标识（`src/main.c` 顶部包含 LicenseRef-Nordic-5-Clause）。

作者
------
该仓库基于 Nordic 官方示例修改而成，主要实现者在 `src/main.c` 中可见实现细节。
