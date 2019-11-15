# Python templates for assignments in book: Small Unmanned Aircraft: Theory and Practice - R. Beard, T. McLain

## Setup instructions
It is recommended to work in virtual environment to avoid version mismatches and dependencies breaks

### create virtual env
```
cd mavsim_python
env_name=mav_env
virtualenv --system-site-packages $env_name
# to show up this env in jupyter notebook
python3 -m ipykernel install --user --name $env_name --display-name "$env_name"
```

### activate env
```
source mav_env/bin/activate
```

### install required packages
```
# requirements added for support till chapter 2. further requirements to be added soon.
pip3 install -r requirements.txt
```