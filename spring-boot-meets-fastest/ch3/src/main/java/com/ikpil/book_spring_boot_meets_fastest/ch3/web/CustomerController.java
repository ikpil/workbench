package com.ikpil.book_spring_boot_meets_fastest.ch3.web;

import com.ikpil.book_spring_boot_meets_fastest.ch3.domain.Customer;
import com.ikpil.book_spring_boot_meets_fastest.ch3.service.CustomerService;
import com.ikpil.book_spring_boot_meets_fastest.ch3.service.LoginUserDetails;
import org.springframework.beans.BeanUtils;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.security.core.annotation.AuthenticationPrincipal;
import org.springframework.stereotype.Controller;
import org.springframework.ui.Model;
import org.springframework.validation.BindingResult;
import org.springframework.validation.annotation.Validated;
import org.springframework.web.bind.annotation.ModelAttribute;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestMethod;
import org.springframework.web.bind.annotation.RequestParam;

import java.util.List;

@Controller // RestController 와 달리 Controller 로 지정한다
@RequestMapping("customers") // RequestMapping 지정한다
public class CustomerController {
    @Autowired
    CustomerService customerService;

    @ModelAttribute
    CustomerForm setUpForm() {
        /**
         * ModelAttribute 애너테이션이 붙으면 메서드 안에서 CustomerForm 클래스를 초기화
         * RequestMapping 보다 먼저 실행 되며, 반환 값은 모델에 자동 추가 된다.
         * 이 클래스가 모델에 자동으로 들어 갈 때, customerForm 으로 들어가서 model 내에서 접근 가능하다
         */
        return new CustomerForm();
    }

    @RequestMapping(method = RequestMethod.GET)
    String list(Model model) { // Spring MVC 에서는 화면에 값을 넘겨주는데 모델 객체를 사용한다
        List<Customer> customers = customerService.findAll();
        model.addAttribute("customers", customers); // 주입

        return "customers/list"; // template/+'뷰 이름' + .html 이 화면 경로가 된다
    }

    @RequestMapping(value = "create", method = RequestMethod.POST)
    String create(@Validated CustomerForm form, BindingResult result, Model model,
                  @AuthenticationPrincipal LoginUserDetails userDetails) {
        /**
         * Validated 애너에티션이 붙으면 Bean Validation 애너테이션이 적용 되고, 그 결과값이 다음 파라메터인
         * BindingResult 인자 result 에 저장 된다
         * result 에 Bean Validation 의 결과가 저장 되었으므로, 에러를 체크 할 수 있다.
         */
        if (result.hasErrors()) {
            return list(model);
        }

        var customer = new Customer();

        /**
         * form 에 저장 되어 있는 값을 customer 객체에 복사한다.
         * 필드 이름과 타입이 같을 때만 데이터가 복사 됨을 주의 해야 한다, 더 유연한 것을 원한다면 Dozer 나 ModelMapper 를 이용해야 한다
         */
        BeanUtils.copyProperties(form, customer);
        customerService.create(customer, userDetails.user);

        /**
         * redirect:url 형식으로 리다이렉트 할 수 있다
         */
        return "redirect:/customers";
    }

    /**
     * RequestParam 애너테이션은 특정 요청 파라미터를 매핑 할 수 있다.
     * 여기선 /customer/edit 고객의 아이디 정보가 된다
     */
    @RequestMapping(value = "edit", params = "form", method = RequestMethod.GET)
    String editForm(@RequestParam Integer id, CustomerForm form) {
        var customer = customerService.findOne(id);

        // 여기서 form 에 고객 정보를 복사하여, html form 에 해당 고격의 정보를 채워 넣는다.
        BeanUtils.copyProperties(customer, form);

        return "customers/edit";
    }

    @RequestMapping(value = "edit", method = RequestMethod.POST)
    String edit(@RequestParam Integer id, @Validated CustomerForm form, BindingResult result) {
        if (result.hasErrors()) {
            return editForm(id, form);
        }

        var customer = new Customer();

        // 편집이기 때문에 HTML form 을 customer 에 셋팅한 후, 업데이트 한다
        BeanUtils.copyProperties(form, customer);
        customer.setId(id);
        customerService.update(customer);

        return "redirect:/customers";
    }

    /**
     * edit 요청이 들어 왔을 때, HTML form 의 name 에 goToTop 값이 있을 경우, 호출 된다
     * 결과는 리다이렉션이다
     */
    @RequestMapping(value = "edit", params = "goToTop")
    String goToTop() {
        return "redirect:/customers";
    }

    @RequestMapping(value = "delete", method = RequestMethod.POST)
    String edit(@RequestParam Integer id) {
        customerService.delete(id);

        return "redirect:/customers";
    }
}
