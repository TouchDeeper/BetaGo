# Control Package for BetaGo 
This is control package for BetaGo.
### PS4 joystick drive the betago
- install ds4drv and connect the ps4, refer to [here](https://retropie.org.uk/docs/PS4-Controller/).
    
    the pair command is `ds4drv --led 000008`, without the `--hidraw` options.
    
- test the connect by `jstest /dev/input/js0`, when you pull the button of ps4, the state of some number will turn from `off` to `on`, this number is corresponding to the button you pull.
- modify the button number of [teleop_ps4.yaml](config/teleop_ps4.yaml) 
## File explanation
None

## Notes
None
## Modify on other project used in BetaGo
None
