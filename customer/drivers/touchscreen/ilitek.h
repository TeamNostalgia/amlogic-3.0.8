
struct ilitek_platform_data{
    u16             x_max;  
    u16             y_max;
    bool    swap_xy;           //define?
    int     irq;
    void (*shutdown)(int on);
    int (*init_gpio)(void);
};






