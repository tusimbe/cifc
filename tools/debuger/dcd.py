import os


path_to_elf = 'cifc.elf'
crashdebug_exe = './CrashDebug'
dump_filename = 'session.log'

if not os.path.exists('cifc.elf'):
    os.system('cp ../../main/build/cifc.elf .')

cmd = "arm-none-eabi-gdb " + path_to_elf + "  -ex \"set target-charset ASCII\" -ex \"target remote | " + crashdebug_exe + " --elf " + path_to_elf + " --dump " + dump_filename + "\""
print(cmd)

os.system(cmd)


    
