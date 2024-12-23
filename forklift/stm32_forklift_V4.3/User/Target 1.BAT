SET PATH=E:\Keil_v5\ARM\ARMCC\Bin;C:\Program Files (x86)\Common Files\Oracle\Java\javapath;G:\PTC\Utilities\MKS Toolkit\bin64;G:\PTC\Utilities\MKS Toolkit\bin;G:\PTC\Utilities\MKS Toolkit\bin\X11;G:\PTC\Utilities\MKS Toolkit\mksnt;C:\Program Files\Microsoft MPI\Bin\;C:\WINDOWS\system32;C:\WINDOWS;C:\WINDOWS\System32\Wbem;C:\WINDOWS\System32\WindowsPowerShell\v1.0\;C:\WINDOWS\System32\OpenSSH\;C:\Program Files (x86)\NetSarang\Xftp 7\;C:\Program Files\Java\jdk-15.0.2\bin；;C:\Program Files (x86)\NetSarang\Xshell 7\;C:\Program Files\dotnet\;C:\Users\Administrator\AppData\Local\Programs\Python\Python37\Scripts\;C:\Users\Administrator\AppData\Local\Programs\Python\Python37\;D:\anaconda3\Scripts\;D:\anaconda3\;D:\anaconda3;D:\anaconda3\Library\mingw-w64\bin;D:\anaconda3\Library\usr\bin;D:\anaconda3\Library\bin;D:\anaconda3\Scripts;D:\python3.7\Scripts\;D:\python3.7\;C:\Users\Administrator\AppData\Local\Microsoft\WindowsApps;C:\Program Files\JetBrains\PyCharm Community Edition 2020.1.3\bin;;E:\odrive can\环境配置包\tup-latest;E:\odrive can\环境配置包\gcc-arm-none-eabi-7-2018-q2-update-win32\bin;E:\odrive can\环境配置包\OpenOCD\0.10.0-13\bin;E:\odrive can\环境配置包\gnu-mcu-eclipse-windows-build-tools-2.12-20190422-1053-win64\GNU MCU Eclipse\Build Tools\2.12-20190422-1053\bin;;E:\VSC\Microsoft VS Code\bin;C:\Users\Administrator\.dotnet\tools
SET ARMCC5_ASMOPT=--diag_suppress=9931
SET ARMCC5_CCOPT=--diag_suppress=9931
SET ARMCC5_LINKOPT=--diag_suppress=9931
SET CPU_TYPE=STM32F103C8
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=Target 1
SET CPU_CLOCK=0x044AA200
"E:\Keil_v5\ARM\ARMCC\Bin\ArmAsm" --Via "..\scp\startup_stm32f10x_md._ia"
"E:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\scp\adc.__i"
"E:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\scp\key.__i"
"E:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\scp\led.__i"
"E:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\scp\dma.__i"
"E:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\scp\delay.__i"
"E:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\scp\sys.__i"
"E:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\scp\usart.__i"
"E:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\scp\main.__i"
"E:\Keil_v5\ARM\ARMCC\Bin\ArmLink" --Via "..\scp\XLZN.lnp"
"E:\Keil_v5\ARM\ARMCC\Bin\fromelf.exe" "..\scp\XLZN.axf" --i32combined --output "..\scp\XLZN.hex"
