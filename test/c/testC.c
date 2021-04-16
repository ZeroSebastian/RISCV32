

void Success() 
{
	asm("ecall");
	while(1);
}

void Failure()
{
	while(1);
}

int main()
{
	Success();
	return 0;
}