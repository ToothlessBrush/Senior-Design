#!/usr/bin/env python3
import subprocess
import os
import sys
import shutil
from pathlib import Path

class Colors:
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    NC = '\033[0m'

def print_colored(text, color):
    print(f"{color}{text}{Colors.NC}")

def run_command(cmd, cwd=None):
    """Run command and return success status"""
    try:
        result = subprocess.run(cmd, shell=True, cwd=cwd, check=True)
        return True
    except subprocess.CalledProcessError as e:
        print_colored(f"Command failed: {cmd}", Colors.RED)
        return False

def verify_toolchain():
    """Verify ARM toolchain installation"""
    print_colored("Verifying ARM toolchain...", Colors.BLUE)
    
    # Check compiler
    if not run_command("arm-none-eabi-gcc --version"):
        print_colored("❌ arm-none-eabi-gcc not found", Colors.RED)
        return False
    
    # Check if standard headers are available
    test_cmd = 'echo "#include <stdint.h>" | arm-none-eabi-gcc -E -'
    if not run_command(test_cmd):
        print_colored("❌ Standard C headers not found (missing newlib?)", Colors.RED)
        return False
    
    # Check other tools
    tools = ["arm-none-eabi-objcopy", "arm-none-eabi-size", "arm-none-eabi-objdump"]
    for tool in tools:
        if not run_command(f"{tool} --version"):
            print_colored(f"⚠️  {tool} not found (optional)", Colors.YELLOW)
    
    print_colored("✅ ARM toolchain verified successfully", Colors.GREEN)
    return True

def main():
    project_root = Path.cwd()
    build_dir = project_root / "build"
    
    # Parse arguments
    clean = "--clean" in sys.argv
    verify = "--verify" in sys.argv
    build_type = "Debug"
    
    if "--release" in sys.argv:
        build_type = "Release"
    
    # Add verification step
    if verify or clean:  # Verify on clean builds or explicit request
        if not verify_toolchain():
            print_colored("Toolchain verification failed. Please install ARM embedded toolchain.", Colors.RED)
            return 1
    
    if clean:
        print_colored("Cleaning build directory...", Colors.YELLOW)
        if build_dir.exists():
            shutil.rmtree(build_dir)
    
    # Create build directory
    build_dir.mkdir(exist_ok=True)
    
    print_colored(f"Configuring project ({build_type})...", Colors.BLUE)
    
    # Configure
    cmake_cmd = f"cmake -DCMAKE_BUILD_TYPE={build_type} -DCMAKE_EXPORT_COMPILE_COMMANDS=ON .."
    if not run_command(cmake_cmd, cwd=build_dir):
        return 1
    
    print_colored("Building project...", Colors.BLUE)
    
    # Build
    if not run_command("cmake --build .", cwd=build_dir):
        return 1
    
    print_colored("Build completed successfully!", Colors.GREEN)
    
    # Copy compile_commands.json to root for clangd
    compile_commands = build_dir / "compile_commands.json"
    if compile_commands.exists():
        shutil.copy(compile_commands, project_root)
        print_colored("compile_commands.json copied to project root", Colors.GREEN)
    
    return 0

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] in ["-h", "--help"]:
        print("Usage: python build.py [--clean] [--release]")
        print("  --clean    Clean build directory before building")
        print("  --release  Build in Release mode (default: Debug)")
        sys.exit(0)
    
    sys.exit(main())
