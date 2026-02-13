# Project Structure

The repository is organized into different folders, each containing relevant examples and resources to help you get started with Paranet.

```
.
├── hello_world
│   ├── adaptative_card (Paranet with adaptive cards)
│   ├── hello_paranet (Simple Paraflow interaction)
│   ├── hello_python_sdk (Paraflow actor using Python SDK)
│   └── hello_sql (Paraflow actor using SQL)
```

Each folder contains specific examples showcasing different capabilities of Paranet. Feel free to explore these to understand how Paranet works and how you can build upon it.

# Project Setup and Execution

## Steps to Run the Project

### Step 1: Deploy the Node
Run the following command to deploy the Paranet node:
```sh
para docker deploy node
```

### Step 2: Deploy the Package
Once the node is successfully created, deploy the package by running:
```sh
para docker deploy package
```

### Step 3: Verify Deployment
Open the following URL in your browser to verify that the Paranet node and all actors are deployed correctly:
[http://localhost:3023/](http://localhost:3023/)






