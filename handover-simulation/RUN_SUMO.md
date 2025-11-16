# How to Run SUMO

## Current File Locations

Your SUMO configuration files are located in:
```
/home/ubuntu/cs297/handover-simulation/
├── urban-scenario.sumocfg    (configuration file)
├── urban-scenario.net.xml    (network file - referenced by sumocfg)
└── urban-scenario.rou.xml    (routes file - referenced by sumocfg)
```

## Running SUMO

### Option 1: Run from the handover-simulation directory (Recommended)

Since your `sumocfg` file uses relative paths (`urban-scenario.net.xml`), you need to run SUMO from the directory where the files are located:

```bash
# Navigate to the directory
cd /home/ubuntu/cs297/handover-simulation

# Run SUMO (use full path to SUMO binary)
/home/ubuntu/cs297/sumo/bin/sumo -c urban-scenario.sumocfg
```

### Option 2: Add SUMO to your PATH

You can add SUMO to your PATH so you can use `sumo` directly:

```bash
# Add to PATH (temporary for current session)
export PATH=/home/ubuntu/cs297/sumo/bin:$PATH

# Then run from the handover-simulation directory
cd /home/ubuntu/cs297/handover-simulation
sumo -c urban-scenario.sumocfg
```

### Option 3: Use absolute paths in sumocfg

If you want to run SUMO from anywhere, you can modify the `sumocfg` file to use absolute paths:

```xml
<input>
    <net-file value="/home/ubuntu/cs297/handover-simulation/urban-scenario.net.xml"/>
    <route-files value="/home/ubuntu/cs297/handover-simulation/urban-scenario.rou.xml"/>
</input>
```

Then you can run:
```bash
/home/ubuntu/cs297/sumo/bin/sumo -c /home/ubuntu/cs297/handover-simulation/urban-scenario.sumocfg
```

## Quick Test

To verify everything works:

```bash
cd /home/ubuntu/cs297/handover-simulation
/home/ubuntu/cs297/sumo/bin/sumo -c urban-scenario.sumocfg --no-step-log
```

This should generate `sumo-trace.xml` in the same directory.

## Finding the SUMO Binary

If the SUMO binary is not in `/home/ubuntu/cs297/sumo/bin/sumo`, find it with:

```bash
find /home/ubuntu/cs297/sumo -name "sumo" -type f
```

Then use the full path to that binary.


