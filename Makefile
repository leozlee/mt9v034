 
#KERN_DIR = /home/jb/work/linux-2.6.32-devkit8500
KERN_DIR = /home/leo/leo/kernel/linux-2.6.32-devkit8500
FILE_NAME=mt9v034
FILE_NAME2=mt9v034_dev
##FILE_NAME2=saa711x

all:
	make -C $(KERN_DIR) M=`pwd` modules 
clean:                            
	make -C $(KERN_DIR) M=`pwd` modules clean
	rm -rf modules.order

obj-m += $(FILE_NAME).o
obj-m += $(FILE_NAME2).o
