# evo usage

## config for paper plot
set figure size by:
```$xslt
evo_config set plot_figsize 2 2
```
## evo_traj
```$xslt
evo_traj tum --ref viwo/groundtruth.csv vio/vio.csv viwo/vio.csv --plot --plot_mode xy --align --save_plot traj.pdf
```

## evo_res
```$xslt
evo_res vio/vio.zip viwo/viwo.zip --use_filenames --plot --save_plot res.pdf
```