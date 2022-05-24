## This script should create a command line Script to collect all the libraries needed in the DEp/python folder.
# It doesnt properly work, check the tutorial again: https://mir.works/blog/2019/8/26/td-summit-external-python



import pathlib

req_file 				= tdu.expandPath(ipar.ExtPython.Pyreqs)
install_target 			= tdu.expandPath(ipar.ExtPython.Target)
install_script_path  	= pathlib.Path(install_target).parents[0]

win_file 		= install_script_path / 'dep_install.cmd'
mac_file 		= install_script_path / 'dep_install.sh'

win_text_op 	= op('text_win')
mac_text_op 	= op('text_mac')

# windows template
win_txt = '''
:: update pip
python -m pip install --user --upgrade pip


:: install from requirements file
py -3.5 -m pip install -r "{reqs}" --target="{target}"
'''

#mac template
mac_txt = '''
#!/bin/bash 

dep=$(dirname "$0")
pythonDir=/python

# change current direcotry to where the script is run from
dirname "$(readlink -f "$0")"

# fix up pip with python3
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
python3 get-pip.py

# Update dependencies
# make sure pip is up to date
python3 -m pip install --user --upgrade pip

# install requirements
python3 -m pip install -r "{reqs}" --target="{target}"
'''

# windows
# clear DAT
#win_text_op.clear()

# format text
formatted_win_txt = win_txt.format(reqs=req_file, target=install_target) 

# write to DAT
#win_text_op.text = formatted_win_txt

# write to file
with open(str(win_file), "w+") as win_script:
	win_script.write(formatted_win_txt)

# mac
# clear DAT
#mac_text_op.clear()

# format text
formatted_mac_txt = mac_txt.format(reqs=req_file, target=install_target) 

# write to DAT
#mac_text_op.text = formatted_mac_txt

# write ot file
with open(str(mac_file), "w+") as mac_script:
	mac_script.write(formatted_mac_txt)
