This demo is a combination of Nvidia's core tutorials and Otonoma's paranet actorization. It uses Nvidia's tutorials to build a simulation with one Jetbot robot and expose 4 actorized skills:

## Actorized Skills

1. **forward** - Makes the Jetbot move forward.
2. **backward** - Makes the Jetbot move backward.
3. **stop** - Stops the Jetbot.
4. **getStatus** - Retrieves the current status of the Jetbot.

## Prerequisites:

### Isaac-sim (compatible with 4.2, 4.5 or 5.0):
- Complete and understand Nvidia's hello world and hello robot tutorials.
- Make sure you can execute [Nvidia's Isaac Sim's Python Environment](https://docs.isaacsim.omniverse.nvidia.com/latest/python_scripting/manual_standalone_python.html#details-how-python-sh-works)

### Otonoma:
- Complete the getting started tasks:
  - [Download and install Guide](https://docs.paranet.otonoma.com/download-install)
  - Ensure you can run `para -V` and get a version displayed.
- Install the Python Paranet SDK for Isaac-Sim:
  
  #### Installation Command:
  - **Linux:**
    ```sh
    cd ~/.local/share/ov/pkg/isaac-sim-<VERSION>/
    ./python.sh -m pip install paranet_agent
    ```
  - **Windows:**
    ```sh
    cd C:\Users\<YourUser>\.local\share\ov\pkg\isaac-sim-<VERSION>\
    python.bat -m pip install paranet_agent
    ```

> **Note:** Replace `<VERSION>` with your Isaac Sim version (e.g., 4.2.0, 4.5.0, 5.0.0). Make sure to install the paranet_agent with your Isaac Sim version.

> **Note:** Make sure to use the path to your Isaac Sim Python Environment.

### Version Requirements:
- Python SDK: v2.0.2 or higher
- Para: v0.21.0 or higher

## Setup:
1. Navigate to the root of the init'ed `hello_robot` directory.
   - **Result should look like:**
     ```
      hello_robot
      ├── actors
      │ └── jetbot.py
      ├── isaac_app.py
      ├── readme.md
      ├── simulation.py
      └── paranet.node.yaml
     ```

## Running the Demo:

1. Run the following commands to build the Paranet Docker project:
   ```sh
   para docker deploy node
   ```
2. Observe the output to ensure the core Paranet services are online. This can also be monitored in the Docker extension.
3. Open the provided Paracord link from the terminal:
   - [http://localhost:3023](http://localhost:3023)
   - This is where we will later trigger skills.
4. Open a terminal at the Isaac-sim installation directory and use Isaac Sim's Python Environment to execute `isaac_app.py`:
   - **Linux:**
     ```sh
     ~/.local/share/ov/pkg/isaac-sim-<VERSION>/isaac-sim.sh isaac_app.py
     ```
   - **Windows:**
     ```sh
     C:\Users\<YourUser\>.local\share\ov\pkg\isaac-sim-<VERSION>\isaac-sim.bat isaac_app.py 
     ```

> **Note:** Replace `<VERSION>` with your Isaac Sim version and make sure to use the path to your Isaac Sim Python Environment.

You should see an Isaac Sim instance with a default ground and a Jetbot spawned.

5. In Paracord, navigate to the "Actor Hub" tab and find the "Jetbot" actor.
6. Test the 4 available skills:
   - `forward`
   - `backward`
   - `stop`
   - `getStatus`

## Stopping the Demo:

1. Stop Isaac Sim by closing the application window
2. (Optional) Clean up Docker resources:
   ```sh
   para docker down
   ```

## Common Issues:

### Docker Network Error:
If you run `para docker deploy node` and get the following error:

```
failed to create network hello-robot-network: Error response from daemon: invalid pool request: Pool overlaps with other one on this address space
Failed to start paranet.
```

Run the following command to remove unused Docker objects:

```sh
docker system prune
```