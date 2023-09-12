// UI Components of Cards_UI for Processing
// author: Lucas Cassiano - cassiano@mit.edu
// date: July 3rd, 2016
// lastUpdate: July 17th, 2016
// version: 1.04
//
// Controllers:
// -Button
// -ImageButton Basic
// -Tooltip
// -Slider
// -Toggle
// -Card
// -Icon (font-awesome icons port)
//
// see https://web.media.mit.edu/~cassiano/projects/cards_ui/index.html
// and https://github.com/lucascassiano/cards_ui
//
// modified by David Tucker 2022

//---
// Color defines for controls

private color c_very_dark = color(36, 37, 46);
private color c_dark = color(29, 33, 44);
private color c_mid = color(44, 58, 71);
private color c_light= color(51, 64, 80);
private color c_hover = color(32, 155, 160);
private color c_text_color = color(255);

//private color c_primary= color(33, 115, 139);
//private color top_right = color(255);

public void uiDark()
{
  c_very_dark = color(36, 37, 46);
  c_dark = color(29, 33, 44);
  c_mid = color(44, 58, 71);
  c_light = color(51, 64, 80);
  c_hover = color(32, 155, 160);
  c_text_color = color(255);
}

public void uiLight()
{
  c_very_dark = color(100);
  c_dark = color(150);
  c_mid = color(200);
  c_light = color(250);
  c_hover = color(32, 155, 160);
  c_text_color = color(10);
}

public color GetBGColor() {
  return c_dark;
}

//---
// Default control sizes

private int s_big = 200;
private int s_height = 30;
private int s_med = 100;
private int s_small = 50;

private int s_textSize = 15;
private int s_textSizeLg = 25;

public int GetLineHeight() {
  return s_height;
}

//---
// Support for mouse clicks

private boolean clicked = false;
private boolean canClick = true;

void mousePressed() {
  clicked = true;
}

void mouseReleased() {
  clicked = false;
  canClick  = true;
}

//---
// Support for text Input/Edit

String bufferText = null;
boolean doneText = false;

void keyPressed() {
  if (keyCode == BACKSPACE) {
    if (bufferText.length() > 0) {
      bufferText = bufferText.substring(0, bufferText.length()-1);
    }
  } else if (keyCode == DELETE) {
    bufferText = "";
  } else if (keyCode != SHIFT && keyCode != ENTER) {
    bufferText = bufferText + key;
  }

  if (keyCode == ' ') {
    bufferText = bufferText.substring(0, bufferText.length()-1);
    bufferText = bufferText + ' ';
  }

  if (keyCode == ENTER) {
    doneText = true;
  }
}

private void EditText(String txt) {
  bufferText = txt;
}

//---
// global variables for Cards
//****Note, should wrap cards into class so multiple cards can be active at once

int card_h = 0;
int card_w = 0;
private int card_x = 0;
private int card_y = 0;

//---

//X and Y are the position of the point of the triangle
public void Tooltip(String text, int x, int y)
{
  int w = max(30, (int)textWidth(text));
  int h = 50;
  int tw = 14; //triangle width
  int th = 15; //triangle height


  //Shadow
  fill(0, 0, 0, 15);
  noStroke();
  rect(5+x-w/2, 5+y-th-h, w, h, 2);
  triangle(5+x-tw/2, 5+y-th, 5+x, 5+y, 5+x+tw/2, 5+y-th);

  //Color
  fill(c_very_dark);
  noStroke();
  rect(x-w/2, y-th-h, w, h, 2);
  triangle(x-tw/2, y-th, x, y, x+tw/2, y-th);

  //Text
  fill(c_light);
  textSize(s_textSize);
  textAlign(CENTER, CENTER);
  text(text, x-w/2, y-th-h, w, h);
}

//---

void Text(String text, int x, int y, int w, int h)
{
  fill(c_light);//c_text_color);
  textSize(s_textSize);
  textAlign(LEFT, CENTER);
  text(text, x, y, w, h);
}

void Text(String text, int x, int y)
{
  Text(text, x, y, s_big, s_height);
}

//---

boolean Button(String text, int x, int y, int w, int h, String tooltip)
{
  if (mouseX >= x && mouseX <= x+w &&
    mouseY >= y && mouseY <= y+h)
  {
    if (tooltip != null)
    {
      Tooltip(tooltip, x+w/2, y-1);
    }

    fill(c_hover);
    stroke(c_light);
    rect(x, y, w, h);

    fill(c_text_color);
    textSize(s_textSize);
    textAlign(CENTER, CENTER);
    text(text, x, y, w, h);

    if (clicked && canClick)
    {
      fill(c_light);
      stroke(c_dark);
      rect(x, y, w, h);

      textSize(s_textSize);
      textAlign(CENTER, CENTER);
      text(text, x, y, w, h);

      canClick = false;
      return true;
    }
  } else
  {
    fill(c_light);
    stroke(c_dark);
    rect(x, y, w, h);

    fill(c_text_color);
    textSize(s_textSize);
    textAlign(CENTER, CENTER);
    text(text, x, y, w, h);

    return false;
  }

  return false;
}

public boolean Button(String text, int x, int y, int w, int h) {
  return Button(text, x, y, w, h, null);
}

boolean Button(String text, int x, int y) {
  return Button(text, x, y, s_med, s_height, null);
}

boolean Button(String text, int x, int y, String t) {
  return Button(text, x, y, s_med, s_height, t);
}

//---

//****FixMe, combined several functions together
// they were identical except for select+padding behaving differently
// see FixMe notes below for differences
// Does select even work? What is it suppose to do.

boolean ImageButton(PImage img, int x, int y, int w, int h, boolean select, int padding)
{
  if (select)
  {
    fill(c_dark);
    stroke(c_light);
    rect(x, y, w, h);

    //****FixMe, pick one
    image(img, x, y, w, h);
    //image(img, x+padding, y+padding, w-2*padding, h-2*padding);
  } else if (mouseX >= x && mouseX <= x+w &&
    mouseY >= y && mouseY <= y+h)
  {
    fill(c_dark);
    stroke(c_light);
    rect(x, y, w, h);

    image(img, x+padding, y+padding, w-2*padding, h-2*padding);

    if (clicked && canClick)
    {
      fill(c_mid);
      stroke(c_mid);
      rect(x, y, w, h);

      //****FixMe, pick one
      image(img, x, y, w, h);
      //image(img, x+padding, y+padding, w-2*padding, h-2*padding);

      canClick = false;
      return true;
    }
  } else
  {
    fill(c_mid);
    stroke(c_mid);
    rect(x, y, w, h);

    //****FixMe pick one
    image(img, x+padding, y+padding, w-2*padding, h-2*padding);
    //image(img, x, y, w, h);

    return false;
  }

  return false;
}

boolean ImageButton(PImage img, int x, int y, int w, int h) {
  return ImageButton(img, x, y, w, h, false, 0);
}

boolean ImageButton(PImage img, int x, int y, int w, int h, int padding) {
  return ImageButton(img, x, y, w, h, false, padding);
}

boolean ImageButton(PImage img, int x, int y, int w, int h, boolean select) {
  return ImageButton(img, x, y, w, h, select, 0);
}

//---

public class TextInput
{
  String text = "";
  boolean active = false;
  String hint = "";
  String label = "";

  public TextInput() {
  }

  public TextInput(String t) {
    this.hint = t;
  }

  public TextInput(String t, String l) {
    this.hint = t;
    this.label = l;
  }

  public String getText() {
    return text;
  }

  //Text Input
  public String draw(int x, int y, int w, int h)
  {
    fill(c_light);
    textSize(s_textSize);
    textAlign(LEFT, BOTTOM);
    text(label, x, y-21, w, 20);

    if (active)
    {
      //Edit Text
      fill(c_dark);
      stroke(c_light);
      rect(x, y, w, h);

      text = bufferText;
      fill(c_text_color);
      textSize(s_textSize);
      textAlign(CENTER, CENTER);
      text(text, x, y, w, h);

      if (mouseX >= x && mouseX <= x+w &&  mouseY >= y && mouseY <= y+h)
      {
        //Inside
      } else
      {
        if (clicked)
        {
          doneText = true;
          //canClick = true;
          active=false;
        }
      }

      if (doneText)
      {
        text = bufferText;
        active = false;
        doneText = false;
      }
    } else if (mouseX >= x && mouseX <= x+w && mouseY >= y && mouseY <= y+h)
    {
      fill(c_hover);
      stroke(c_light);
      rect(x, y, w, h);

      fill(c_text_color);
      textSize(s_textSize);
      textAlign(CENTER, CENTER);
      text(text, x, y, w, h);

      if (clicked && canClick)
      {
        fill(c_light);
        stroke(c_dark);
        rect(x, y, w, h);

        fill(c_light);
        textSize(s_textSize);
        textAlign(CENTER, CENTER);
        text(text, x, y, w, h);

        EditText(text);
        canClick = false;
        active = true;
      }
    } else
    {
      fill(c_light);
      stroke(c_dark);
      rect(x, y, w, h);

      fill(c_text_color);
      textSize(s_textSize);
      textAlign(CENTER, CENTER);
      text(text, x, y, w, h);

      active = false;
    }

    if (text.length() == 0)
    {
      fill(150);
      textSize(s_textSize);
      textAlign(CENTER, CENTER);
      text(hint, x, y, w, h);
    }
    return text;
  }
}

//---

public boolean Toggle(boolean value, int x, int y, int w, int h)
{
  fill(c_dark);
  stroke(c_light);
  rect(x, y, w, h, h/2);

  int pos = 0;
  if (value)
    pos = w-h;
  //Hover
  if (mouseX >= x && mouseX <= x+w && mouseY >= y && mouseY <= y+h)
  {
    fill(red(c_hover), green(c_hover), blue(c_hover), 100);
    noStroke();
    ellipse(x+h/2+pos, y+h/2, h-2, h-2);

    fill(c_hover);
    noStroke();
    ellipse(x+h/2+pos, y+h/2, h-8, h-8);

    if (clicked && canClick) {
      value = !value;
      canClick = false;
      return value;
    }
  }
  //Normal
  else
  {
    fill(c_light);
    stroke(c_light);
    ellipse(x+h/2+pos, y+h/2, h-8, h-8);
  }

  return value;
}

public boolean Toggle(boolean value, int x, int y, int w) {
  return Toggle(value, x, y, w, s_height);
}

public boolean Toggle(boolean value, int x, int y) {
  return Toggle(value, x, y, s_small, s_height);
}

//****FixMe, w/h not really doing anything here
// do we even need it at all?
public boolean Toggle(String text, boolean value, int x, int y, int w, int h)
{
  fill(c_light);
  textSize(s_textSize);
  textAlign(LEFT, CENTER);
  text(text, x+s_small+20, y, w, h);

  return Toggle(value, x+10, y, s_small, s_height);
}

public boolean Toggle(String text, boolean value, int x, int y, int w) {
  return Toggle(text, value, x, y, w, s_height);
}

public boolean Toggle(String text, boolean value, int x, int y) {
  return Toggle(text, value, x, y, s_small, s_height);
}

//---

// tooltip is '%' (input 0-1, displayed as 0-100%), '#' (any input, displayed as int) or ' ' (no tooltip)
public float Slider(float min, float max, float value, int x, int y, int w, int h, char tooltip)
{

  fill(c_light);
  noStroke();
  rect(x, y+h/2, w, 4, 2);

  float pos = map(value, min, max, 0, w);

  fill(c_hover);
  noStroke();
  rect(x, y+h/2, pos, 4, 2);

  //Hover
  if (mouseX >= x && mouseX <= x+w && mouseY >= y && mouseY <= y+h)
  {
    if (mousePressed)
    {
      //stroke(c_hover); //****FixMe
      pos = mouseX;
      value = map(pos, x, x+w, min, max);

      fill(red(c_hover), green(c_hover), blue(c_hover), 100);
      noStroke();
      ellipse(pos, y+h/2, h, h);

      fill(c_hover);
      noStroke();
      ellipse(pos, y+h/2, h-8, h-8);

      //Tooltip
      if (tooltip=='%') {
        String s = (int)(value*100)+"%";
        Tooltip(s, (int)(pos), y);
      } else if (tooltip=='#') {
        String s = str((int)value);
        Tooltip(s, (int)(pos), y);
      }
      // else do nothing
    } else
    {
      fill(red(c_hover), green(c_hover), blue(c_hover), 50);
      noStroke();
      ellipse(pos+x, y+h/2, h, h);

      fill(c_hover);
      noStroke();
      ellipse(pos+x, y+h/2, h-8, h-8);
    }
  }
  //Normal
  else
  {
    fill(c_hover);
    noStroke();
    ellipse(pos+x, y+h/2, h-8, h-8);
  }

  return value;
}

public float Slider(String label, float min, float max, float value, int x, int y, int w, int h, char tooltip) {
  int w2 = 0;
  int pad = 15;
  float tw = textWidth(label);

  fill(c_light);
  textSize(s_textSize);
  textAlign(LEFT, CENTER);
  text(label, x, y, tw, h);

  w2 = (int)(w-tw-pad);

  return Slider(min, max, value, (int)(tw+x+pad), y, w2, h, tooltip);
}

//****FixMe, clean this up, there are far too many variatons

public float Slider(float min, float max, float value, int x, int y, int w, int h) {
  return Slider(min, max, value, x, y, w, h, ' ');
}

public float Slider(String label, float min, float max, float value, int x, int y, int w, int h) {
  return Slider(label, min, max, value, x, y, w, h, ' ');
}

public float Slider(float value, int x, int y) {
  return Slider(0f, 1f, value, x, y, s_big, s_height, ' ');
}

public float Slider(float value, int x, int y, char tooltip) {
  return Slider(0f, 1f, value, x, y, s_big, s_height, tooltip);
}

public float Slider(String label, float value, int x, int y) {
  return Slider(label, 0f, 1f, value, x, y, s_big, s_height, ' ');
}

public float Slider(String label, float value, int x, int y, char tooltip) {
  return Slider(label, 0f, 1f, value, x, y, s_big, s_height, tooltip);
}

public float Slider(String label, float value, int x, int y, int w, int h) {
  return Slider(label, 0f, 1f, value, x, y, w, h, ' ');
}

public float Slider(float value, int x, int y, int w, int h) {
  return Slider(0f, 1f, value, x, y, w, h, ' ');
}

public float Slider(float value, int x, int y, int w, int h, char tooltip) {
  return Slider(0f, 1f, value, x, y, w, h, tooltip);
}

public float Slider(float min, int max, int value, int x, int y, int w, int h) {
  return Slider((float) min, (float) max, value, x, y, w, h, ' ');
}

//---

//--Font Awesome Icons--
// based on: https://github.com/encharm/Font-Awesome-SVG-PNG
// this Method loads the .svg files from /svg/ folder
// more icons can be added at the folder, and called by its names

//This hashmap is used to avoid reload the same icon multiple times
HashMap<String, PShape> usedIcons = new HashMap<String, PShape>();

//****FixMe, add icon support to image button
public void Icon(String name, int x, int y, int w, int h)
{
  if (usedIcons.get(name)==null)
  {
    try
    {
      PShape i= loadShape(name+".svg");
      usedIcons.put(name, i);
      shape(i, x, y, w, h);
    }
    catch(Exception e)
    {
      println("CARD_UI - ERROR: svg icon not found");
    }
  } else
  {
    PShape i = usedIcons.get(name);
    shape(i, x, y, w, h);
  }
}

//---

public class DropDown
{
  private String iconDown = "angle-down";
  private String iconUp = "angle-up";
  private boolean open = false;

  public DropDown()
  {
  }

  public int draw(String args[], int selected, int x, int y, int w, int h)
  {
    fill(c_light);
    stroke(c_mid); //c_light
    rect(x, y, w, h);

    fill(c_text_color);
    textSize(s_textSize);
    textAlign(CENTER, CENTER);
    text(args[selected], x, y, w, h);

    if (Button(" ", x+w, y, h, h))
    {
      open = !open;
    }

    if (!open)
    {
      Icon(iconDown, x+w+1, y+1, h-2, h-2);
    } else
    {
      Icon(iconUp, x+w+1, y+1, h-2, h-2);

      fill(c_light);
      stroke(c_mid); //c_light
      rect(x+h, y+h, w, s_height*args.length);

      for ( int i=0; i<args.length; i++)
      {
        int ix = x+h;
        int iy = y+h+s_height*i;

        boolean highlighted = (mouseX >= ix && mouseX <= ix+w && mouseY >= iy && mouseY <= iy+h);
        if (highlighted)
        {
          fill(c_hover);
          stroke(c_mid); //c_light
          rect(ix, iy, w, h);

          if (clicked && canClick)
          {
            selected = i;
            open = false;
            canClick = false;
          }
        }

        fill(c_text_color);
        textSize(s_textSize);
        textAlign(CENTER, CENTER);
        text(args[i], ix, iy, w, h);
      }
    }

    return selected;
  }

  public int draw(String args[], int selected, int x, int y, int w) {
    return draw(args, selected, x, y, w, s_height);
  }

  public int draw(String args[], int selected, int x, int y) {
    return draw(args, selected, x, y, s_big, s_height);
  }
}

//---

public void beginCard(String card_title, int x, int y, int w, int h)
{
  //Shadow
  fill(0, 0, 0, 15);
  noStroke();
  rect(x+5, y+5, w, h);

  fill(c_light);
  noStroke();
  rect(x, y, w, 40, 2, 2, 0, 0);

  fill(c_text_color);
  textSize(s_textSize);
  textAlign(CENTER, CENTER);
  text(card_title, x, y, w, 40);

  fill(c_mid);
  noStroke();
  rect(x, y+40, w, h-40, 0, 0, 2, 2);

  card_h = h-40;
  card_w = w;
  card_x = x;
  card_y = y+40;
}

public void beginCard(int x, int y, int w, int h)
{
  fill(c_mid);
  noStroke();
  rect(x, y, w, h);

  card_h = h;
  card_w = w;
  card_x = x;
  card_y = y;
}

public void endCard()
{
  card_h = 0;
  card_w = 0;
  card_y = 0;
  card_x = 0;
}

//---

//****Note code below is suspect

/*--------------------------------------------------------------------------------+
 |This sketch contains all unstable new methods and classes of cards_ui UIkit      |
 |author: Lucas Cassiano                                                           |
 |creation date: August 7, 2016                                                    |
 |last update: August 7, 2016                                                      |
 ----------------------------------------------------------------------------------*/

//Drag&Drog Object
//Needs a Spawn object (also template), a receiver and a
public void Draggable() {
}

public class DraggableObject {
  private int originX = -1;
  private int originY = -1;
  private boolean selected = false;
  private int x=0, y=0, w=0, h=0;
  private boolean hover = false;
  private boolean setOriginPos = false;
  private int originPosX =0; //original Positions
  private int originPosY = 0;
  private boolean hasNewOrigin = false;
  private int gridSize = 1;

  private void setOrigin(int x, int y) {
    originPosX = x;
    originPosY = y;
  }
  public DraggableObject() {
    //Empty Constructor
  }

  public DraggableObject(int x, int y, int w, int h) {
    this.x = x;
    this.y = y;
    this.w = w;
    this.h = h;
  }

  public void drag(int x, int y, int w, int h) {
    if (!setOriginPos) {
      if (!hasNewOrigin) {
        this.x = x;
        this.y = y;
        setOrigin(x, y);
        this.w = w;
        this.h = h;
      } else {
        setOrigin(this.x, this.y);
      }

      setOriginPos = true;
    }

    if (clicked && selected) {
      this.x = originPosX-originX+mouseX;
      this.y = originPosY-originY+mouseY;
      hasNewOrigin = true;
    } else {
      selected=false;
      setOriginPos = false;
    }

    if (mouseX >= this.x && mouseX <= this.x+w &&
      mouseY >= this.y && mouseY <= this.y+h) {
      if (clicked && !selected) {
        selected = true;
        originX = mouseX;
        originY = mouseY;
      }

      hover = true;
    } else {
      hover = false;
    }
  }

  public boolean getSelected() {
    return selected;
  }
};

public class DropprableObject {
  private int x, y, w, h;
  private StringList canReceive = new StringList();

  public void setPosition(int x, int y) {
    this.x = x;
    this.y = y;
  }

  //Verifies if the point _x,_y is inside the rectangle of the droppableObject
  public boolean Contains(int _x, int _y) {
    if (_x >= this.x && _x <= this.x+w &&
      _y >= this.y && _y <= this.y+h) {
      return true;
    } else {
      return false;
    }
  }
}

/*--UI elements--*/
public class DragDropCard extends DraggableObject {
  public DragDropCard() {
    super();
  }

  public void BeginCard(String title, int x, int y, int w, int h) {
    super.drag(x, y, w, y+40);
    if (super.getSelected() || super.hover)
      beginCard( title+" hover", super.x, super.y, w, h);
    else {
      beginCard( title, super.x, super.y, w, h);
    }
  }
};

//Basic Text Button
public boolean Draggable(int x, int y, int w, int h) {
  if (mouseX >= x && mouseX <= x+w && mouseY >= y && mouseY <= y+h)
  {
    fill(c_hover);
    stroke(c_light);
    rect(x, y, w, h);

    if (clicked)
    {
      fill(c_light);
      stroke(c_dark);
      rect(x+mouseX, y+mouseY, w, h);

      //canClick = false;
      //line(mouseX, 20, mouseX, 80);
      return true;
    }
  } else
  {
    fill(c_light);
    stroke(c_dark);
    rect(x, y, w, h);

    return false;
  }

  return false;
}
