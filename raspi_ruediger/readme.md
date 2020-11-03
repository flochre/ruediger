cd {repo_rudiger}/raspi_ruediger/
catkin_make

good infos : https://github.com/UbiquityRobotics/ubiquity_launches

3. The very first thing we will do is change both the host name
   and the account password.

   Under linux, each machine is given a unique name (e.g. tobor,
   my_robot, jerky, etc.) on your local network.  In addition,
   Linux supports multiple user accounts on the machine.  Each
   user account has its own password.  For the system image on
   the micro-SD card that you plugged into the Raspi2, the initial
   machine name is `robot` and the account name is `ros`.  We
   need to change `robot` to something else and we need change
   the password from `rosrobot` to something else.

   First think of a new password.  You might as well pick a pretty
   good password, since we are going to set things up so that you
   do not have to type it in very often.  A good password usually
   consists of a combination of upper and lower case letters
   along with at least one digit and one puncutation character.
   
   In order to change the password, run the following command:

        sudo passwd

   You will be prompted for the original password (e.g. `rosrobot`)

        [sudo] password for robot:

   And you will type in the new password in twice.  The password
   characters will not show on the screen as yout type them in:

        Enter new UNIX password: 
        Retype new UNIX password: 

   If you get `Sorry, passwords do not match`, please try again.

   You also need to pick a new name for the robot.  In this case
   we recommend that you specify a robot name with all lower case
   letters.  You will be typing this robot name occasionally, so
   do not make it too hard on yourself.  In the command below,
   replace `NEW_HOSTNAME` with the name you selected:

        sudo echo "NEW_HOSTNAME" > /etc/hostname
        hostname
        # The new robot host name should print out; if not try again

   Now reboot the robot:

        sudo reboot

   This will cause the robot to reboot.

4. After wait for about a minute, please repeat the steps 1 and 2
   to log into the robot using your `NEW_HOSTNAME`:

        ping -c 5 NEW_HOSTNAME.local
        ssh ubuntu@NEW_HOSTNAME.local
        Password:
	# Type in the new password.

5. We are going to set up secure shell on the robot.  There are two
   steps here.  First, we will generate public/private key pair
   for your robot. Second, we will make it so that you can run
   `ssh` between your machine and itself without prompting you
   for a password.  This is called a recursive ssh and it does
   occasionally happen.

   First, we will create the public/private key pair:

        ssh-keygen -t rsa

   Second, we will make sure that we can do the recursive call to `ssh`:

        ssh-copy-id ros@NEW_HOSTNAME.local
        # You will prompted for your new password

   Now we verify that we can do the recursive `ssh`:

        ssh ros@NEW_HOSTNAME.local

   If you are prompted for a password, something has gone wrong and you
   should restart this step from the beginning.

   If you did not get prompted for a password, you succeeded and you can
   simply type the following command to close out the `ssh` connection:

        exit

6. Now we need to log out from robot and do two more `ssh` configuration
   on your desktop/laptop.  The purpose of this configuration is to make
   for sure that `ssh` is not always prompting you for passwords.

   First, we exit the robot:

        exit

   Now we generate a public/private key pair for you laptop/desktop:

        ssh-keygen -t rsa

   Now we make sure that we can recursivly connecto to ourselves.
   Notice thae we are using accent graves (i.e. back qutoes (\`)
   rather than single quotes (') in the command below

        ssh-copy-id `whoami`@`hostname`.local

   If you get prompted for a password, provide the password for your
   desktop/laptop.

   Now verify that you can recursively `ssh` from desktop/laptop,
   to itself.

        ssh `hostname`.local

   If you are not prompted for a password, you succeeded.  If you are
   prompted for a password, something went wrong and you need to try
   the `ssh-copy-id` command again.  Type:

        exit

   To close the recursive connection.

   We are almost done.  Now we want to be able to log into your
   robot without a password.  Do the following:

        ssh-copy-id ros@NEW_HOSTNAME.local

   where `NEW_HOSTNAME` is the hostname you picked for your robot.
   If you are prompted for a password, use the new password you
   created for the robot.

   Verify that you can login into the robot without a password.

        ssh ros@NEW_HOSTNAME.local

   As usual, you do not want to be promted for a password.  Type
   the following command to disconnect.

        exit

That covers the initial setup for now.