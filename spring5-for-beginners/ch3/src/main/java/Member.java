import exception.WrongPasswordException;
import lombok.Getter;
import lombok.NonNull;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

import java.time.LocalDateTime;

@Getter
@Setter
@RequiredArgsConstructor
public class Member {
    private Long id;
    @NonNull
    private String email;
    @NonNull
    private String password;
    @NonNull
    private String name;
    @NonNull
    private LocalDateTime registerDateTime;

    public void changePassword(String oldPassword, String newPassword) {
        if (!password.equals(oldPassword)) {
            throw new WrongPasswordException();
        }
        this.password = newPassword;
    }
}

