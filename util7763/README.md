
Some simple utilities to preview how some LED routines will look before we hook them up to a robot.

These need work to be more plug-and-play with the actual robot code, this is a work in progress.

To test out the display (which uses Java Swing/AWT guis):

Default primary/secondary colors are RED and GREEN.

Some effects can take one or more colors.

```bash

javac LedStripDemo.java

java LedStripDemo solid
java LedStripDemo solid magenta
java LedStripDemo flash yellow
java LedStripDemo strobe orange
java LedStripDemo waverb
java LedStripDemo wave
java LedStripDemo wave blue black
java LedStripDemo fire
java LedStripDemo pacman

```

