
# Basic Linux Commands

## File and Directory Operations
1. **List files and directories**:  
   ```bash
   ls
   ```
   Options:  
   - `ls -l`: List with details (permissions, size, etc.).  
   - `ls -a`: Show hidden files.  
   - `ls -lh`: List with human-readable file sizes.

2. **Change directory**:  
   ```bash
   cd [directory]
   ```
   Examples:  
   - `cd /home/user`: Go to a specific directory.  
   - `cd ..`: Move up one directory.  
   - `cd ~`: Go to the home directory.

3. **Create a directory**:  
   ```bash
   mkdir [directory_name]
   ```

4. **Remove a file or directory**:  
   ```bash
   rm [file_name]
   rm -r [directory_name]
   ```
   - `rm -r`: Removes directories and their contents recursively.  
   - **Caution**: Use with care; this action is permanent.

5. **Copy files or directories**:  
   ```bash
   cp [source] [destination]
   ```
   - Add `-r` for directories: `cp -r source_dir dest_dir`.

6. **Move or rename files**:  
   ```bash
   mv [source] [destination]
   ```

## IDE
1. **open in VS Code current directory**:  
   ```bash
   code .
   ```

1. **open in VS codium current directory**:  
   ```bash
   codium .
   ```
   
## File Content Operations
1. **View file content**:  
   ```bash
   cat [file_name]
   ```
   - `cat file.txt`: Display the entire content.

2. **Search within a file**:  
   ```bash
   grep [search_term] [file_name]
   ```
   - Example: `grep "error" logs.txt`

3. **Display first/last lines of a file**:  
   ```bash
   head [file_name]
   tail [file_name]
   ```

4. **Edit a file**:  
   Use editors like `nano`, `vim`, or `gedit`:  
   ```bash
   nano [file_name]
   ```

## System Information
1. **Display current directory**:  
   ```bash
   pwd
   ```

2. **Check disk usage**:  
   ```bash
   df -h
   du -sh [file_or_directory]
   ```


## Permissions and Ownership
1. **Change file permissions**:  
   ```bash
   chmod [permissions] [file_name]
   ```
   Example: `chmod 755 script.sh`

2. **Change file ownership**:  
   ```bash
   chown [owner]:[group] [file_name]
   ```


## Package Management
1. **Install a package** (Ubuntu/Debian):  
   ```bash
   sudo apt install [package_name]
   ```

2. **Update package lists**:  
   ```bash
   sudo apt update
   ```

3. **Upgrade installed packages**:  
   ```bash
   sudo apt upgrade
   ```

---

*This list covers fundamental commands for beginners. Explore their manual pages (`man [command]`) for more options.*
