namespace MauiSandbox.ViewModels;

public class SecretEventViewModel : IQueryAttributable
{
    public string Text { get; }
    public DateTime Date { get; }

    public void ApplyQueryAttributes(IDictionary<string, object> query)
    {
        throw new NotImplementedException();
    }
}