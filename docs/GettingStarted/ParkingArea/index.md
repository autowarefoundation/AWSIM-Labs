# PARKING AREA DEMO

_After completing the Quick Start demo, you can proceed to this section._

1. Download the hd-map and pointcloud map of the parking area.

    [Parking Are Map](https://drive.google.com/drive/folders/1ly8Z1_SQc9bJ834t_zKCyYSTyKJgj8ho?usp=sharing)

2. Download the latest release from:

    [AWSIM Labs GitHub Releases Page](https://github.com/autowarefoundation/AWSIM-Labs/releases){.md-button .md-button--primary}
     and launch `awsim_labs.x86_64`.

3. From the Map option, select **'Parking Area'**.

4. Set the Ego position to _**'81580.52, 50083.58, 41'**_ and rotation to _**'0,0,100'**_ for the starting point. You can later change the position by providing a 2D pose in the RViz2 screen to update the location.
5. Uncheck the Traffic Control box.

6. Start the simulation by clicking the **'Load'** button.

7. Now you can use the [e2e_simulator](https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/QuickStartDemo/) or [scenario simulator](https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/UsingOpenSCENARIO/).
# Scenarios:
Four simple scenarios have been created using the [scenario editor](https://github.com/tier4/scenario_simulator_v2) for use in the Parking Area environment. Before the scenarios can be used with the scenario simulator, the tutorial provided [here](https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/UsingOpenSCENARIO/) must be completed.
Below are illustrated explanations of the scenarios.

_The map named 'Parkin_Area_ss2' should be selected from map selection._


!!! Warning

    The titles can be clicked to download the scenarios.


[**Test 1: (L parking next to vehicle)**
](https://drive.google.com/file/d/1734TvS6G1IiF3dbVNyrzXm3pQFtAGnj9/view?usp=drive_link)

![1.png](1.png)

[**Test 2: (L parking between 2 vehicles)**
](https://drive.google.com/file/d/1rNGgOTXZO_X-FmLKAkTBaitx2r2jnYO3/view?usp=drive_link)

![2.png](2.png)

[**Test 3: (Angled parking next to vehicle)**
](https://drive.google.com/file/d/1pTGwo2SbF6yFIvij6n9iSkjIYZVd6EsJ/view?usp=drive_link)

![3.png](3.png)

[**Test 4: (Parallel parking)**
](https://drive.google.com/file/d/1--UMYejSgnzD_gwW2JpB8kUtMRin48iV/view?usp=drive_link)

![4.png](4.png)

# Dimensions of Parking Areas

The dimensions of the parking spaces were made based on this reference:

   - [https://cdn-wordpress.webspec.cloud/intrans.iastate.edu/uploads/sites/15/2018/12/Chapter_08-2018.pdf](https://cdn-wordpress.webspec.cloud/intrans.iastate.edu/uploads/sites/15/2018/12/Chapter_08-2018.pdf)
   - [https://www.dimensions.com/element/30-degree-parking-spaces-layouts](https://www.dimensions.com/element/30-degree-parking-spaces-layouts)


**Parallel Parking:** 3m x 7.275m   _(The dimensions were set to 1.5 times the size of the Lexus vehicle.)_

![paralel_p.png](paralel_p.png)

**L Parking:** 2.5m x 6m

![l_p.png](l_p.png)

**Angled parking:** 7m x 3m  &  60-degree

![angeled_p.png](angeled_p.png)
