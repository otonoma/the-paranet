# Introduction
This kit demonstrates the basics of node to node federation, where each node exists on the same local network.

When deployed, 4 nodes are created. 3 of the "color" variety, and 1 of the "painter" variety. The objective is to determine what colors are available to the painter, and then create a blended color with the cooperation of the nodes. This is a simple narrative that focuses on highlighting the needed scaffolding, and node interaction syntax. A multi node system like this is not required to accomplish the narrative goal... it is just a vehicle to demonstrate concepts. 

# Project Setup and Execution

## Steps to Run the Project

This kit expects the user to have already run the basic hello_world examples.

### Step 0: Ensure Devkit login
Run the following command to ensure devkit resource access. 
(This is not required on every run, but is a good step to start a work session.)
```sh
para devkit login
```

### Step 1: Deploy the Nodes and Packages
Run the following command to deploy the Paranet nodes:
```sh
bash scripts/setup.sh
```
If everything deploys properly, you'll see this in the terminal:
```
Project Hello federate started successfully
Done
--------------------------------
 ID       PORT  PROJECT       PACKAGES       
 blue     3024  node-blue     hello-federate 
 painter  3026  node-painter  hello-federate 
 red      3023  node-red      hello-federate 
 yellow   3025  node-yellow   hello-federate 
RED: http://localhost:3023
BLUE: http://localhost:3024
YELLOW: http://localhost:3025
PAINTER: http://localhost:3026
```

### Optional Step 2: Manually Re-Deploy the Packages
If you experiment with adjustments to the paraflow, you can redeploy the packages without needing to set everything up again:
```sh
bash scripts/install-packages.sh
```

### Step 3: Verify Deployment
Open the displayed URL's in your browser to verify that the Paranet node and all actors are deployed correctly.
(http://localhost:3023 -> 3026)

### Optional Step 4: Shutdown
The project can be shut down with the following command:
```sh
bash scripts/teardown.sh
```

## Discussion:

### Using the demo
- Pilot the demo from the "painter" node. (http://localhost:3026) Consider the "color" nodes as seperate buisness or functional units. You have federated to them for help, but you do not have complete control of their behavior. 
- All nodes contain a system event trigger that resets the nodes into a "demo ready" state. This occurs when setup.sh is run.
    - Color nodes have their inventory of paint reset.
    - Painter node makes an initial test connection to all federated nodes, and stores what colors are available to it.
- Optionally run "pallete/list_all_colors" from paracord to review some info about the color nodes, and the color they provide.
- Run "pallete/color_blend" to accomplish the demo goal. Enter 2 colors as determined by "list_all_colors". (default: red, yellow, blue)
    - If inventory exists for both colors, the resultant blended color will be displayed.
    - In the background, the inventory for the included colors has been decremented. If you run "color_blend" enough times, you'll see the workflow fails due to lack of inventory on the relevant color node. Note this default inventory can be adjusted in `/color/actors/color_tools.paraflow`
- Particulars of the demo (such as reseting inventory or reseting available colors) can be adjusted with skills like "color_tools/reset_inventory" and "pallete/reset_available_colors". 
- Reminder that adjustments made to paraflow generally require `bash scripts/install-packages.sh` to be run. Adjustments to tables or the node deployments require `bash scripts/teardown.sh` , then `bash scripts/setup.sh` to be run. 

### Demonstrated Infrastructure
- The demo highlights the particulars of multiple distinct nodes deployed on a local network. With additional config (and obviously a cloud VM or similar) one or more of the nodes could be deployed remotely, and federation is still possible. 

- For ease of use, multi node/package deployment is handled by the contents of the `scripts` directory. Take some time to node the details there, such as:
    - assigned ports
    - locations of specified node, and package config yaml files
    - the specification of federation, especially the `bidirectional` flag
    - in `install-packages.sh` note the inclusion of an ENV var (blue, red, yellow)
        - in `color/paranet.yaml` note the definition of that variable. This makes it available to the actor.
        - in `color/actors/color_tools.paraflow` note the `constant $SELF = Env("COLOR");`. This makes the color available to the workflow itself.

