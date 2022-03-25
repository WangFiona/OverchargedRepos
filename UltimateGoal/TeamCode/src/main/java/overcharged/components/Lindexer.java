package overcharged.components;

///Lindexer component, includes movement functions
public class Lindexer {
    ///Servo which drives the Lindexer
    private OcServo slide;

    ///Preset postitions for the Lindexer
    private float slideUp = 97/255f;
    public static float slideDown = 173/255f;
    private float slidePos1 = 147/255f;
    private float slidePos2 = 127/255f;

    public Lindexer(OcServo slide){
        this.slide = slide;
    }

    ///Method which moves the slide up
    public void slideUp(){
        slide.setPosition(slideUp);
    }

    ///Method which moves the slide down
    public void slideDown(){
        slide.setPosition(slideDown);
    }

    ///Method which moves the slide to the first powershot position
    public void powershot1(){
        slide.setPosition(slidePos1);
    }

    ///Method which moves the slide to the second powershot position
    public void powershot2(){
        slide.setPosition(slidePos2);
    }

}
