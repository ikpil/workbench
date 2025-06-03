namespace MauiSandbox.Views;

public partial class HomePage : ContentPage
{
	int count = 0;

	public HomePage()
	{
		InitializeComponent();
	}
	
	private void ContentPage_NavigatedTo(object sender, NavigatedToEventArgs e)
	{
		SecretEventsCollection.SelectedItem = null;
	}
}

