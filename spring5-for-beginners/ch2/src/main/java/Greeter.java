
public class Greeter {
    private String format;

    public String greet(String quest) {
        return String.format(format, quest);
    }

    public void setFormat(String format) {
        this.format = format;
    }
}
