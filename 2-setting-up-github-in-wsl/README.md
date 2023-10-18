# Setting Up Github in WSL
1. Set up Credentials 
```bash 
git config --global user.name "yourusername"
git config --global user.email "youremail@gmail.com"
```
2. Create key and copy key
```bash
ssh-keygen -t rsa -b 4096 -C "youremail@gmail.com"
cat ~/.ssh/id_rsa.pub
```
3. Paste key in Github `Settings > SSH and GPG keys > New SSH key`
4. Clone repo and ready to go! 