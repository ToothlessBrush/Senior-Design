# GDB initialization file for STM32F411 Flight Controller

# Enable TUI with source-only layout (no assembly)
tui enable
layout src
focus cmd

# Disable pagination for smoother output
set pagination off

# Auto-refresh TUI layout on stops (prevents corruption on remote targets)
define hook-stop
refresh
end

# Optional: Auto-refresh after common commands
define hook-next
refresh
end

define hook-step
refresh
end

define hook-continue
refresh
end

define hook-finish
refresh
end

# ARM Cortex-M specific settings
set mem inaccessible-by-default off

# Pretty printing
set print pretty on
set print array on

# Target connection (uncomment if you want auto-connect)
# target extended-remote localhost:3333

# Load symbols (uncomment if you want auto-load)
# file build/FlightController.elf

# Useful breakpoint shortcuts
# Example: break main
# Example: break HardFault_Handler
