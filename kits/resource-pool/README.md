# Resource Pool Example

This example demonstrates how to solve the problem of allocating resources for tasks in a general way.  For example, you have two runners that can fetch items from the warehouse for customers.  When you get a new request for an item from the warehouse, you need to choose an available runner (possibly waiting both are busy), and then assign them to the job.  The resource manager actor in this example handles this type of
scenario with the concept of a resource pool.  For the runner example, we would create a pool called "runners" and add two members to the pool representing the two runners.  When new requests arrive, we can
ask the manager to "checkout" a member of the "runners" pool, i.e. assign a runner for exclusive use for this request.  When the request is complete, we "checkin" the assigned member to return the runner to the
availability pool for outstanding or future requests.

## Deploy

Make sure you are logged with

```para devkit login```

Deploy node:

```para docker deploy node```

Deploy package:

```para docker deploy package```

## Demo actor

This example includes a demo actor that illustrates how to use the resource manager.  Use the demo actor to start a "test_flow", which represents an example process that consists of three independent jobs that require a robot to complete.  The robots must be allocated from a "robots" pool which consists of two robots.  Since the process requires three robots, you will see two jobs start right away, and the third will
not start until one of the first two finishes making a robot available for the third job.
