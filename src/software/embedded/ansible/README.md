### Tag guide 

Ansible tags group tasks together so we can run or skip certain groups of tags in a playbook.
-  (ex. using the **dependencies** group runs only the tasks for setting up dependencies on the Pi)

Add an existing tag to a new task if that task *must* run for that tag group, even if the task is an indirect prerequisite. 
- (ex. **enable_password_less_sudo_for_rsync.yml** has the **dependencies** tag even though it imports no dependencies since it grants access to passwordless sudo for rsync file transfers, which is needed for configuring dependencies)


### All Tags
- dependencies : setting up dependencies for the Pi
- configure_pi : config tasks for Pi
- set_hostname :
- update_hostname :
- add_user_robot_to_dialout :
- check_internet :
- setup_wifi_interface :
- rtt : for computer-robot RTT statistics
- reboot : 
- shutdown : 
- always : Ansible special tag; runs unless explicitly skipped.
- never : Ansible special tag; task will not run unless specifically requested.




