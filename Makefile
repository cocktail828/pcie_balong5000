KDIR := /home/lester/source/kernel
obj-m = pcie_balong.o pcie_weth.o pcie_cdev.o
pcie_balong-y = pcie_balong_dev.o pcie_balong_debug.o
pcie_cdev-y = pcie_cdev.o pcie_cdev_rc.o pcie_cdev_desc.o pcie_cdev_dbg.o  hisi_adp_pcdev_rc.o  pcie_cdev_test.o  pcie_cdev_ai_test.o #pcie_usb_relay.o
pcie_weth-y = wan_eth_rc.o wan_eth_table_rc.o wan_eth_event_buffer.o
all:
	make -C $(KDIR) M=$(PWD) modules

clean:
	make -C $(KDIR) M=$(PWD) clean
