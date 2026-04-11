/* [Drop down box:] */
// combo box for number
Numbers=2; // [0, 1, 2, 3]

// combo box for string
Strings="foo"; // [foo, bar, baz]

/* [Slider] */
// slider widget for number
slider =34; // [10:100]
cube_size =10; // [10:100]

//step slider for number
stepSlider=2; //[0:5:100]

/* [Checkbox] */
//description
Variable = true;

/* [Textbox] */
// Text box for string with length 8
String="length"; // [8]

/* [Special vector] */
Vector3=[12,34,46]; //[0:2:50]

/* [Hidden] */
debugMode = true;

module __Customizer_Limit__ () {}
shownAfterBrace = true;


cube([cube_size, cube_size, cube_size]);

if (Variable) {
    translate([0, 0, cube_size])
        sphere(slider);
}