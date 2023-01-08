import org.springframework.context.annotation.AnnotationConfigApplicationContext;

public class Main {

    public static void main(String[] args) {
        var ctx = new AnnotationConfigApplicationContext(AppContext.class);
    }
}
