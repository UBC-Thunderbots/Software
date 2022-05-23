# Qt5toPyQt5
Use Qt5 Widget on PyQt5 Application


Run the following commands:

```bash
python configure.py
make 
sudo make install
```

Then you should copy the file `PyAnalogClock.so` that is inside modules to the side of your script, in my case in the same folder of main.py.

	.
	├── main.py
	└── PyAnalogClock.so
       
Example:

```bash
cd Example
python main.py
```
 
Output: 


![](Screenshot.png) 