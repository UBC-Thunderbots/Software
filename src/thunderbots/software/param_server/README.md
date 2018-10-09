# Dynamic Reconfigure

The param server node initializes the dynamic_reconfigure server and calls the set_parameter service with the proper configuration message created when all `parameters` are constructed

### Registering a parameter to be configured through rqt_reconfigure
~~~python
navigator = gen.add_group("Navigator")
~~~
The namespace in dynamic_parameter.cpp should correspond to the group defined in `Params.cfg`.
Calling `add` on the group object will put the paramter in that group. 

~~~python
navigator.add("int_example", int_t, 0, "An example int parameter", 50,  0, 100)
~~~

The arguments are as follows (min and max are only for double and int)
~~~python
("name", type, level, "description", default, min, max)
~~~

### Running the GUI 
After launching ai, opening the rqt_reconfigure node will open the gui
~~~
rosrun rqt_reconfigure rqt_reconfigure
~~~
![rqt_reconfigure](rsc/gui.png?raw=true "rqt_reconfigure")

