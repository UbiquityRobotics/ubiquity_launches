
# Tree
```
Running Configure	
	[0]: Exit without save
	[1]: Change current hostname
	[2]: Manage hostname WiFi access points
		[0]: Exit WiFi access point mode
		[1-n]: Edit/delete ssid
			[0] Done editing this Wifi access point
			[1] Edit WiFi Name (currently '{0})
			[2] Change Wifi Password
			[3] Delete entire '{0}' access point)
			[X]: Invalid Command
		[n+1]: Add new Wifi access point
		[X]: Invalid Command
	[3]: Manage secure shell access
		[0]: Exit SSH mode
		[1-n]: Set up SSH keys for machine
		[X]: Invalid Command
	[4]: Save everything and exit
	[X]: Invalid Command
Exit
```

# States

```
    states = {
        '0' : {'default' : exit()},
        '1' : {'default' : change_hostname()},
        '2' : {
            'default' : manage_wifis(),
            '0' : up_one(),
            '1' : {
                'default' : edit_wifi(),
                '0' : up_one(),
                '1' : edit_ssid(),
                '2' : change_pass(),
                '3' : delete_wifi()
            }
            '2' : add_wifi()
        },
        '3' : {
            'default' : manage_ssh(), 
            '0' : up_one(),
            '1' : setup_ssh()
        },
        '4' : {'default' : save_exit()}
    }
```

