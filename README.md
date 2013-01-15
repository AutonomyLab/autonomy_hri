To migrate from `autonomy_human` to `autonomy_hri`

```bash
roscd autonomy_human
cd ..
mv autonomy_human autonomy_hri
cd autonomy_hri
git remote set-url origin git@github.com:AutonomyLab/autonomy_hri.git 
[or]
git remote set-url origin https://github.com/AutonomyLab/autonomy_hri.git
git pull
```

