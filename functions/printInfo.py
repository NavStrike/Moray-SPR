from colorama import init, Fore, Style, Back

init(autoreset=True)

def print_info(msg):
    print(f"{Fore.GREEN}{Style.BRIGHT}{msg}{Style.RESET_ALL}")

def print_warning(msg):
    print(f"{Fore.YELLOW}{Style.BRIGHT}WARNING: {msg}{Style.RESET_ALL}")

def print_error(msg):
    print(f"{Fore.RED}{Style.BRIGHT}ERROR: {msg}{Style.RESET_ALL}")

def print_debug(msg):
    print(f"{Fore.CYAN}{Style.BRIGHT}DEBUG: {msg}{Style.RESET_ALL}")
