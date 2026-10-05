# silvanus_bottle_picker
The robot that pays for itself
## Perception: Bottle Recognition & Approach
**Bottle Recognition & Approach** 

This section documents the iterative development of the vision and approach module for the Silvanus robot.

### Update: [01/10/26] - Environment Setup & First Detections

* **Environment Setup:** Successfully initialized and configured the workspace for the bottle detection algorithms.
* **Initial Model Testing:** Deployed the first approximate model for bottle recognition.
![Evnironment for bottle detection test](images/bottle_test_1.png)
*Caption:Environment detection in gazebo.*
* **Troubleshooting:** Encountered a deployment blocker where the model was `Not working due to a GPU error` (specifically: *Problem by setting up with the CUDA environmentand unable to run YOLO*.
* **Validation:** After resolving the GPU issue, successfully tested the model using random sample images.

#### Detection Results:
*(Below are the initial detection tests after resolving the GPU error)*


![First bottle detection test](images/bottle_test_2.jpg)
*Caption: Additional testing to verify model accuracy on random images.*