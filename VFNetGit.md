# Instructions for using Git and other Web resources on the VFNet

Most of the machines on the Virtual Factory Network (VFNet) are not directly connected to the Internet, and cannot browse the Web, access Git repositories, use `apt-get` to update Linux software, or do anything else that requires a direct connection to the Internet. A special NIST firewall is configured to block all traffic to and from machines on the VFNet. A few machines are allowed through the firewall, such as the Linux PC `eisner`. These can be used with some caveats to move information between the VFNet and the Internet.

The purpose of the VFNet is to provide an isolated network for conducting research using software or equipment that is not allowed on the NIST network. Using `eisner` as a proxy to the Internet via port forwarding, or other means of passing VFNet traffic through `eisner` as an intermediary, is not allowed, Nevertheless, occassional access to web resources by VFNet machines is useful, and some provisions have been made to allow this. 

## Using Secure Shell (`ssh`)

`ssh` can be used to establish a secure encrypted connected between two machines, for interactive terminal sessions, running software remotely, or copying files. 

To establish a simple terminal session, do:

`ssh <user>@<host>`

where `<user>` is your username on the remote `host`. If your username is omitted, your local username is used, but will result in an error if there is no account for you on the remote `host`. 

Passing the `-X` option enables forwarding of any remote program graphical output to the local system using the X Windows protocol. For example, user `you` could set up a X session using this: 

`ssh -X you@eisner`

and then run programs from the resulting prompt, such as a Firefox web browser for reading documentation on a local VFNet machine. To run such a program directly without establishing an interactive session, you can do this: 

`ssh -X you@eisner firefox`

To copy files between your local and remote account file systems, use `scp`, e.g., 

`scp original.txt you@eisner:copy.txt`

This will copy `original.txt` from the local machine into your home directory on `eisner`, calling it `copy.txt`. Omitting the destination name (but retaining the colon `:`) will copy the file using its original name. If your accounts are the same, a file copy can be done in either direction like this: 

```
scp file.txt eisner:
scp eisner:file.text .
```

## Using GitHub

GitHub repositories, such as the `youbot` repository in the `usnistgov` organization, can be accessed indirectly using Git's bare repository feature. A bare repository is a repository containing only Git's administrative files, cloned from an upstream repository. Git commands that affect working files can't be used on the bare repo, since there are no working files in it. You can push from a bare repository, and fetch to it, but you can't pull or merge to it. The purpose of a bare repository is to act as an intermediate place to hold Git actions from downstream repositories that can't access the upstream repository directly. The disadvantage is that it's a two-step process: pushes to it from downstream (inside the VFNet) won't appear on GitHub until a second push from the bare repo is done. Likewise, a fetch must be done on the bare repo to get any changes from GitHub, before they can be pulled into the downstream VFNet repo. 
To set this up for the GitHub `youbot` repo, from local VNNet machine `polaris` using `eisner` as the intermediary, do the following:

1. On `eisner`, 
```
cd <path/to/your/eisner/repos>
git clone --bare git@github.com:usnistgov/youbot.git
```
Note the `--bare` option here. 

2. On `polaris`,
```
cd <path/to/your/polaris/repos>
git clone eisner:<path/to/your/eisner/repos>/youbot.git
```
This is a normal clone, without the `--bare` option.

3. Still on `polaris`, check out the branch you will be using, or create a new one. To see the branches that are available (on the GitHub remote), do:

`git branch -a`

You will see something like this: 
```
* master
  remotes/origin/HEAD -> origin/master
  remotes/origin/master
  remotes/origin/collabdemo
```
Let's say you want to work in the `collabdemo` branch. Check it out:

`git checkout collabdemo`

3. Still on `polaris`, edit your files as usual, and when done, do the usual `git add <new files>` and `git push`. This will record the commits in the `collabdemo` branch on the uptream bare repository. These won't appear on GitHub yet. 

4. On 'eisner', do a 'git push'

## Go to a folder of your choice on the bare repo machine 1. On the
interim remote (bare repo): git clone --bare
git@github.com:usnistgov/youbot.git

Go to a folder of your choice on the local machine 2. On the client
remote: git clone rcandell@eisner:github/youbot

make your changes on local machine

Pn the local machine * Add your changes using git add * Commit your
changes * Push the changes: 3. git push


For down stream changes.  Changes made elsewhere * refresh bare repo
git fetch origin master:master

* pull to the local machine git pull


This file uses Markdown (`.md`) format. See [help.github.com/articles/markdown-basics](http://help.github.com/articles/markdown-basics)
