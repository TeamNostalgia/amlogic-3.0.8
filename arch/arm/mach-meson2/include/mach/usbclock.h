#ifndef __MACH_MESON2_USB_CLK_H
#define __MACH_MESON2_USB_CLK_H

#define USB_PHY_PORT_A      0
#define USB_PHY_PORT_B      1

#define USB_PHY_MODE_HW        (0)
#define USB_PHY_MODE_SW_HOST      (1)
#define USB_PHY_MODE_SW_SLAVE        (2)

int set_usb_phy_clk(int rate);
int set_usb_phy_id_mode(unsigned int port,unsigned int mode);

#endif /* __MACH_MESON2_USB_CLK_H */
