SED:=$(shell which sed || type -p sed) -i -e
define KCONFIG_ENABLE_OPT
       $(SED) "/\\<$(1)\\>/d" $(2);
       echo "$(1)=y" >> $(2);
endef

define KCONFIG_SET_OPT
       $(SED) "/\\<$(1)\\>/d" $(3);
       echo "$(1)=$(2)" >> $(3);
endef

define KCONFIG_DISABLE_OPT
       $(SED) "/\\<$(1)\\>/d" $(2);
       echo "# $(1) is not set" >> $(2) ;
endef


ifeq ($(filter eclipse_defconfig Install_uImage eclipse_default,$(MAKECMDGOALS)),)
-include $(KBUILD_OUTPUT)/Makefile

include $(KBUILD_OUTPUT)/.config
else
.PHONY:$(KBUILD_OUTPUT)/.config eclipse_default
endif
eclipse_default: 
	$(MAKE)  all
	$(MAKE) uImage
$(KBUILD_OUTPUT)/.config:
	mkdir -p $(@D)
	$(MAKE) $(ECLIPSE_CONFIG_NAME)_defconfig	
	
ifneq   ($(DISABLE_LIST)$(ENABLE_LIST),)
	$(foreach i,$(DISABLE_LIST),$(call KCONFIG_DISABLE_OPT,CONFIG_$(i),$@))
	$(foreach i,$(ENABLE_LIST),$(call KCONFIG_ENABLE_OPT,CONFIG_$(i),$@))
endif
	yes ' ' | make  oldconfig
eclipse_defconfig:$(KBUILD_OUTPUT)/.config

Install_uImage:
	sync_sdcard /media/kernel $(O)/arch/arm/boot/uImage
