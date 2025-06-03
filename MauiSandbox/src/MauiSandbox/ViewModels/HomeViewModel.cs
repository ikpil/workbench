using System.Collections.ObjectModel;
using System.Windows.Input;

namespace MauiSandbox.ViewModels;

public class HomeViewModel : IQueryAttributable
{
    public ObservableCollection<SecretEventViewModel> SecretEventViewModels { get; }
    
    public ICommand NewSecretEventCommand { get; }
    public ICommand SelectSecretEventCommand { get; }

    public void ApplyQueryAttributes(IDictionary<string, object> query)
    {
        throw new NotImplementedException();
    }
}