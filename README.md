# Sensor Feedback Limitations - Application in a 4-Machine Power System

This repository presents the methods and simulation results from [cite]. 

Content

# main_analys.m

This script loads the linearized model by calling the function load_linear_model.m

## load_linear_model.m

- Loads a saved linearized model of the Kundur 4-machine 2-area test system.
- Transfer functions used for optimal control design are generated.
 
Note that the original model contains a lot of obsolete states. Ideally these are removed by taking the minimal realisation (matlab function minreal.m). But due to the condition of the model, this generate a significant amount of numerical error, making the control design infeasible. This however, is mainly an issue with the minreal function. To avoid this we instead compute the reduced-order approximation using the built in balred function. This is done in modelred_hsv.m where we have made it easier for the user to select the desired reduced order system size.

If another linearized model is desired (for instance if you want to investigate another load flow scenario) then you will have do some "manual engineering" and revisit the model reduction steps.

## plot_filtering_sensitivity.m

Shows a plot of the filtering sensitivety [cite]

## Control design

- Synthesize a H2 controlelr and a PSS-style controller 
- Compare linear step responses. Note that the real nonlinear simulation will differ quite a bit.
- The choise of PSS gain is obtained by running a root locus. Choose k_pss so that damping of the inter-area mode becomes 10%
- Compare disturbance response ratio between PSS and H2.

### Other measurements

Create H2 controllers for
- Complementary external freqeuncy measurement (with time delay)
- Line flow measurement
- Voltage measurement

These are compared with the base line H2 controller using local pahse angle measurement, and the uncontrolled system.

# Main Simulation

Main file to run nonlinear simulations and plot results

Choises
- Model
- PSS control gain adjustemt. Originaly tuned down to 20% to give more interesting simulations. Other ways of reducing stability can be to replace the impednace loads with active power loads.
- load predifned controllers?
- Innertia of generators. Here we consider 75% of the Kundur case just to give more interesting simulations.
- Distrubance step size. (-353 selected)

## Run simulations or load from saved file
