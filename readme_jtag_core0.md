mettre à jours le script .... avec 

```
# Usage with Vitis IDE:
# In Vitis IDE create a Single Application Debug launch configuration,
# change the debug type to 'Attach to running target' and provide this 
# tcl script in 'Execute Script' option.
# Path of this script: C:\Users\marti\Documents\Robot\1-SOFTWARE\Eurobot_2025\Zynq_SW\Eurobot_2025_system\_ide\scripts\systemdebugger_eurobot_2025_system_standalone.tcl
# 
# 
# Usage with xsct:
# To debug using xsct, launch xsct and run below command
# source C:\Users\marti\Documents\Robot\1-SOFTWARE\Eurobot_2025\Zynq_SW\Eurobot_2025_system\_ide\scripts\systemdebugger_eurobot_2025_system_standalone.tcl
# 
connect -url tcp:127.0.0.1:3121
targets -set -nocase -filter {name =~"APU*"}
rst -system
after 3000
targets -set -filter {jtag_cable_name =~ "Digilent JTAG-HS2 210241802783" && level==0 && jtag_device_ctx=="jsn-JTAG-HS2-210241802783-13722093-0"}
fpga -file C:/Users/marti/Documents/Robot/1-SOFTWARE/Eurobot_2025/Zynq_SW/opossum_core2/_ide/bitstream/Zynq_block_design_wrapper.bit
targets -set -nocase -filter {name =~"APU*"}
loadhw -hw C:/Users/marti/Documents/Robot/1-SOFTWARE/Eurobot_2025/Zynq_SW/Zynq_block_design_wrapper/export/Zynq_block_design_wrapper/hw/Zynq_block_design_wrapper.xsa -mem-ranges [list {0x40000000 0xbfffffff}] -regs
configparams force-mem-access 1
targets -set -nocase -filter {name =~"APU*"}
source C:/Users/marti/Documents/Robot/1-SOFTWARE/Eurobot_2025/Zynq_SW/opossum_core2/_ide/psinit/ps7_init.tcl
ps7_init
ps7_post_config
targets -set -nocase -filter {name =~ "*A9*#0"}
dow C:/Users/marti/Documents/Robot/1-SOFTWARE/Eurobot_2025/Zynq_SW/opossum_core1/Debug/opossum_core1.elf
targets -set -nocase -filter {name =~ "*A9*#1"}
dow C:/Users/marti/Documents/Robot/1-SOFTWARE/Eurobot_2025/Zynq_SW/opossum_core2/Debug/opossum_core2.elf
configparams force-mem-access 0

# --- AMP "comme le robot final" ---
# On ne fait PAS de "con" (resume) sur A9#1 ici : le binaire opossum_core2.elf
# est bien telecharge en DDR (a 0x10080000, cf lscript.ld / CPU1_ENTRY_ADDR
# dans main.c), mais le coeur CPU1 reste a l'arret (halte JTAG). C'est
# ensuite le reveil logiciel de CPU0 (ecriture 0xFFFFFFF0 + sev(), dans
# opossum_common/main.c) qui doit le faire demarrer -- exactement comme lors
# d'un boot standalone depuis la flash.
#
# ATTENTION : un coeur a l'arret via JTAG n'est pas dans le meme etat qu'un
# coeur fraichement resete et parque par la boot ROM en WFE. Le sev() de
# CPU0 NE reveillera PAS ce CPU1 halte par le debogueur. Une fois que le
# message "[CPU0] Reveil du CPU1..." s'affiche, il faut resumer CPU1 a la
# main (voir resume_cpu1.tcl, ou la vue Debug de Vitis) pour simuler le
# reveil materiel.

targets -set -nocase -filter {name =~ "*A9*#0"}
con
``` 
# sur vitis 
Lance xsct (terminal Vitis ou xsct dans le PATH), puis :
```source path_vers_le_fichier/Eurobot_2025/Zynq_SW/Eurobot_2025_system/_ide/scripts/systemdebugger_eurobot_2025_system_standalone.tcl```
